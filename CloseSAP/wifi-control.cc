/*
Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.
    * Neither the name of The Linux Foundation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "wifi-control.h"
#include "wifi-control-private.h"
#include "config-keys.h"
#include "qcmap-control.h"
#include "utils/utils.h"
#include "wifi-scan.h"

#include <adk/wakelock.h>
#include <adk/config/config.h>
#include <adk/ipc/service.h>
#include <adk/ipc/serviceemitter.h>

#include <data/QCMAP_Client.h>

#include <unistd.h>
#include <iostream>
#include <regex>
#include <string>
#include <tuple>

#include <boost/assign/list_of.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/algorithm/string.hpp>

#define ADK_LOG_BACKEND ADK_LOG_BACKEND_STDOUT
#define ADK_LOG_LEVEL adk_log_level::DEBUG
#include "adk/log.h"

using adk::config::Config;

using adk::msg::AdkMessage;
using adk::msg::AdkMessageService;

//  Update Wi-Fi advanced parameters
#define UPDATE_WIFI_ADVANCED_PARAMS 1

namespace adk {
namespace connectivity {
namespace wifi {

// Wi-Fi interfaces for various modes
// TBD: this might change when we support dual radio

// STA mode is always over wlan0 interface
const char *kStaInterface          = "wlan0";

// Scan is always over wlan0 interface
const char *kScanInterface         = "wlan0";

// Onboard Soft-AP is always over wlan1 interface
const char *kOnboardAPInterface    = "wlan1";

// MLAN Soft-AP is over wlan1 interface if HomeSta is also active
const char *kMlanAPWithHomeStaInterface    = "wlan1";

// MLAN Soft-AP is over wlan0 interface if HomeSta is not active
const char *kMlanAPWithoutHomeStaInterface = "wlan0";

// Ethernet interface
const char *kEthernetInterface = "eth0";

//  Wi-Fi Control service
WiFiControl::WiFiControl(
    AdkMessageService *connectivity_manager,
    const std::shared_ptr<ConnectivityQueue<ConnectivityCommand>> &queue)
    : data_(new Private(connectivity_manager, queue)) {
    data_->qcmap_control_ = std::unique_ptr<QCMAPControl>(new QCMAPControl(queue));

  ADK_LOG_DEBUG("WiFiControl::WiFiControl Started()");
}

WiFiControl::~WiFiControl() {
  if (data_->rx_thread_.joinable()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Destructor called before stopping RX thread");
    Stop();
  }
  ADK_LOG_DEBUG("Wi-Fi-Control::WiFiControl Ended()");
}

bool WiFiControl::Init(const std::shared_ptr<adk::connectivity::nsm::NetworkStateMachine> &state_machine) {

  ADK_LOG_DEBUG("Wi-Fi-Control::Init()");

  // Store the NSM object pointer
  data_->state_machine_ = state_machine;

  data_->connectivity_wifi_config_ =
      adk::config::Config::Create(CONFIG_PATH_CONNECTIVITY_WIFI);
  if (data_->connectivity_wifi_config_ == nullptr) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to open db file: %s ",
                  CONFIG_PATH_CONNECTIVITY_WIFI);

    return false;
  }

  data_->device_config_ = adk::config::Config::Create(CONFIG_PATH_DEVICE);
  if (data_->device_config_ == nullptr) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to open db file: %s",
                  CONFIG_PATH_DEVICE);

    return false;
  }
  data_->connectivity_states_config_ =
      adk::config::Config::Create(CONFIG_PATH_CONNECTIVITY_STATES);
  if (data_->connectivity_states_config_ == nullptr) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to open db file: %s",
                  CONFIG_PATH_CONNECTIVITY_STATES);

    return false;
  }

  data_->connectivity_ethernet_config_ =
      adk::config::Config::Create(CONFIG_PATH_CONNECTIVITY_ETHERNET);
  if (data_->connectivity_ethernet_config_ == nullptr) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to open db file: %s",
                  CONFIG_PATH_CONNECTIVITY_ETHERNET);

    return false;
  }

  // Create a backup of the /etc/misc/wifi/hostapd.conf and /etc/misc/wifi/sta_mode_hostapd.conf
  // configuration files during the very first boot
  std::string  cmd;

  cmd = "adk_wifi_config_update";
  cmd += " backup_orig_hostapd_conf";
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
    return false;
  }

  cmd = "adk_wifi_config_update";
  cmd += " backup_orig_sta_mode_hostapd_conf";
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
    return false;
  }

  // Reset the Ethernet connected state and wait for Ethernet grace period so that
  // the state will be updated with the Ethernet cable plugged-in status during init
  data_->ethernet_connected_ = false;

  // Reset the Wi-Fi STA mode connected flag during init
  data_->sta_connected_ = false;

  //  Start Wi-Fi Control listener thread
  data_->rx_thread_ = std::thread([this]() { data_->ControlRXThread(); });

  if (!data_->qcmap_control_->QCMAPControlStatus()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to setup QCMAP Control");
    return false;
  }

  //  Enable QCMAP/MobileAP client first.
  if (!data_->qcmap_control_->enableMobileap()) {
    ADK_LOG_ERROR("Wi-Fi-Control::** QCMAP ENABLE FAILED - EXITING");
    return false;
  }

  //  Set NAT type to restrict the port numbers configured by QCMAP
  if (!data_->qcmap_control_->setNATTypeToPortRestricted()) {
    ADK_LOG_ERROR("Wi-Fi-Control::** QCMAP Set NAT Type failed - EXITING");
    return false;
  }

  // Wait for few seconds to check if Ethernet cable is plugged-in or not
  int ethernet_check_timeout_sec = 0;

  data_->connectivity_ethernet_config_->Read(
      &ethernet_check_timeout_sec,
      adk::Connectivity::keys::wifi::kEthernetUpTimeout);

  if (ethernet_check_timeout_sec) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Wait %d sec for Ethernet detection", ethernet_check_timeout_sec);

    // Initiliase the network state machine once we have detected whether the Ethernet cable is plugged in or not
    data_->timer_task_ = Timer::create()->createTask([this]() { data_->InitStates(); },
                                                     std::chrono::seconds{ethernet_check_timeout_sec});
  } else {
    // Initalise the states immediately if no timeout is configured
    data_->InitStates();
  }

  return true;
}

// Perform Initialisation of network state machine
void WiFiControl::Private::InitStates() {
  bool wifi_enabled;

  ADK_LOG_DEBUG("Wi-Fi-Control::InitStates()");

  // Read the Wi-Fi enable status from the configuration
  if (!GetEnableState(&wifi_enabled)) {
    ADK_LOG_ERROR("Wi-Fi-Control::InitWiFi():Wi-Fi state read failed");
    return;
  }

  // Try to enable Wi-Fi if it was enabled before
  if (wifi_enabled) {
    ADK_LOG_DEBUG("Wi-Fi-Control::InitStates():Wi-Fi enabled");

    //  Wi-Fi is enabled in configuration, so enable during init
    state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiEnableEventID);

  } else {
    //  Wi-Fi was disabled before, so keep it disabled during init
    ADK_LOG_DEBUG("Wi-Fi-Control::InitStates():WiFi disabled");
    state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiDisableEventID);
  }
}

void WiFiControl::Exit() {
  // Send exit command to RX thread
  ConnectivityCommand connectivity_command;
  connectivity_command.command = CONNECTIVITY_MANAGER_EXIT_CMD;
  data_->connectivity_queue->add(connectivity_command);

  // Wait for thread to exit
  if (data_->rx_thread_.joinable()) {
    data_->rx_thread_.join();
  }
}

void WiFiControl::Private::ControlRXThread() {
  ADK_LOG_INFO("Wi-Fi-Control::ControlRXThread()");

  bool running = true;

  //  Wait for the Wi-Fi control messages in the connectivityQueue
  while (running) {
    //  Get the input Wi-Fi control commands from the connectivity queue
    system_wake_lock_toggle(false, kWakelockWifi, 0);
    received_command_ = connectivity_queue->remove();
    system_wake_lock_toggle(true, kWakelockWifi, 0);

    ADK_LOG_INFO("Wi-Fi-Control::command received: %d",
                 received_command_.command);

    switch (received_command_.command) {

      // Exit command from connectivity manager (internal)
      case CONNECTIVITY_MANAGER_EXIT_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Exit command");
        running = false;
        break;
      }

      //  Enable Wi-Fi
      case CONNECTIVITY_WIFI_ENABLE_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi enable command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiEnableEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Disable Wi-Fi
      case CONNECTIVITY_WIFI_DISABLE_CMD:  {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi disable command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiDisableEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi onboarding
      case CONNECTIVITY_WIFI_ONBOARD_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi onboard command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiOnboardEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi scan
      case CONNECTIVITY_WIFI_SCAN_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi scan command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiScanEventID);
        break;
      }

      //  Wi-Fi STA connect
      case CONNECTIVITY_WIFI_CONNECT_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi connect command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiConnectEventID);
        break;
      }

      //  Wi-Fi complete onboarding
      case CONNECTIVITY_WIFI_COMPLETE_ONBOARDING:  {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi complete onboarding command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiCompleteOnboardingEventID);
        break;
      }

      // Wi-Fi STA connected event from QCMAP
      case CONNECTIVITY_WIFI_EVENT_STA_CONNECTED: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi connected QCMAP indication");

        // Send the connected indication to ADK-IPC bus for other ADK managers
        std::string ssid;
        GetStaConnectionSSID(&ssid);
        SendConnectStatus(true, ssid);

        sta_connected_ = true;

        //  Reset timer waiting for connection response
        ResetControlTimer();

        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiConnectedEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      // Wi-Fi STA Disconnected event from QCMAP
      case CONNECTIVITY_WIFI_EVENT_STA_DISCONNECTED: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi disconnected QCMAP indication");

        // Send the disconnected indication to ADK-IPC bus for other ADK managers/upper layers (IoTivity/AllPlay)
        std::string ssid;
        GetStaConnectionSSID(&ssid);
        SendConnectStatus(false, ssid);

        sta_connected_ = false;

        // NSM is not currently processing Wi-Fi disconnected event
        // state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiDisconnectedEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi MLAN Onboarding start
      case CONNECTIVITY_WIFI_MLAN_ONBOARDING_START: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi onboarding start command");
         state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiMlanOnboardingStartEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi MLAN start lead (MLAN AP enable)
      case CONNECTIVITY_WIFI_MLAN_START_LEAD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi MLAN start lead command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiMlanStartLeadEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi get MLAN AP credentials
      case CONNECTIVITY_WIFI_MLAN_GET_CREDENTIALS: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi MLAN get credentials command");

        std::string ssid;
        std::string password;

        //  Get MLAN AP credentials
        if (GetMlanAPCredentials(&ssid, &password)) {
          ADK_LOG_DEBUG("Wi-Fi-Control::Get MLAN AP credentials successful");
          SendMlanAPGetCredentialsStatus(true, ssid, password);
        } else {
          ADK_LOG_ERROR("Wi-Fi-Control::Set MLAN AP credentials failed");
          SendMlanAPGetCredentialsStatus(false, "", "");
        }
        break;
      }

      //  Wi-Fi MLAN start slave
      case CONNECTIVITY_WIFI_MLAN_START_SLAVE: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi MLAN start slave command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiMlanStartSlaveEventID);
        break;
      }

      //  Wi-Fi MLAN stop
      case CONNECTIVITY_WIFI_MLAN_STOP: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi MLAN stop command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiMlanStopEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      //  Wi-Fi MLAN Onboarding stop
      case CONNECTIVITY_WIFI_MLAN_ONBOARDING_STOP: {
        ADK_LOG_DEBUG("Wi-Fi-Control::ControlRXThread():Wi-Fi MLAN onboarding stop command");
        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiMlanOnboardingStopEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      // Ethernet Connected event from QCMAP
      case CONNECTIVITY_ETHERNET_CONNECTED: {
        ADK_LOG_DEBUG("Wi-Fi-Control::Ethernet connected event");
        SendEthernetConnected();

        // Set the Ethernet connected state
        ethernet_connected_= true;

        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMEthernetConnectedEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      // Ethernet Disconnected event from QCMAP
      case CONNECTIVITY_ETHERNET_DISCONNECTED: {
        ADK_LOG_DEBUG("Wi-Fi-Control::Ethernet disconnected event");
        SendEthernetDisconnected();

        // Reset the Ethernet state state
        ethernet_connected_= false;

        state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMEthernetDisconnectedEventID);

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      // Get connectivity network info
      case CONNECTIVITY_GET_NETWORK_INFO_CMD: {
        ADK_LOG_DEBUG("Wi-Fi-Control::Get network info command");

        // Send the current network info
        SendNetworkInfo();
        break;
      }

      default: {
        ADK_LOG_ERROR("Wi-Fi-Control::Command not yet supported: %d",
                      received_command_.command);
        break;
      }
    }
  }
  ADK_LOG_DEBUG("Wi-Fi-Control::RxThread exiting");
}

// Wi-Fi disable
void WiFiControl::Disable() {
  ADK_LOG_DEBUG("Wi-Fi-Control::Disable()");

  if (data_->PerformDisable()) {
    // Update the state and send the status message
    data_->SendDisableStatus(true);
    data_->SetEnableState(false);
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::disable failed");
    data_->SendDisableStatus(false);
  }
}

// Scan for neighbouring APs
void WiFiControl::Scan() {
  ADK_LOG_DEBUG("Wi-Fi-Control::Scan()");
  const char *scan_interface = kScanInterface;
  std::vector<ScanInfo> in_scan_list;

  //  Perform Wi-Fi scan
  if (!performWiFiScan(scan_interface, &in_scan_list)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi scan failed");
    data_->SendScanResults(false, in_scan_list);
    return;
  }

  ADK_LOG_DEBUG("Wi-Fi control::Wi-Fi scan successful");
  printWiFiScanlist(in_scan_list);

  data_->SendScanResults(true, in_scan_list);
}

// Enable Wi-Fi in onboarding mode
void WiFiControl::EnableInOnboardMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::EnableInOnboardMode()");

  if (data_->PerformOnboarding()){
    //  New onboarding command received - update the "onboarded" state
    data_->SetOnboardState(false);

    //  Send the response
    data_->SendOnboardingStatus(true);

  } else {
    data_->SendOnboardingStatus(false);
  }
}

// Wi-Fi connect
void WiFiControl::Connect() {
  std::string ssid="";
  std::string passphrase="";
  bool homeAP = false;
  bool wps = false;

  ADK_LOG_DEBUG("Wi-Fi-Control::Connect()");

  //  New connect request, reset the Wi-Fi timer
  data_->ResetControlTimer();

  //  Check if wps flag key exists
  if (data_->received_command_.payload_adk_ipc.connectivity_wifi_connect()
           .has_wps()) {
    wps = data_->received_command_.payload_adk_ipc.connectivity_wifi_connect().wps();
  }

  // Check if there's input SSID or password
  if (data_->StaConnectGetCommonParams(data_->received_command_.payload_adk_ipc, &ssid, &passphrase)) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Using SSID and password in ADK-IPC Wi-Fi connect command");
  } else if (wps) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Using WPS connect");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::No SSID/password or WPS in ADK-IPC Wi-Fi connect command");
    data_->SendConnectStatus(false, ssid);
    return;
  }

  if (!data_->StaConnect(ssid, passphrase, wps)) {
    //  New connection failed, initiate a connection to homeAP
    //  if it was connected before
    ADK_LOG_ERROR("Wi-Fi-Control::STA connect attempt failed");
    data_->StartStaReconnectToHomeAP();
  }
}

//  Wi-Fi start timer waiting for onboarding completion
void WiFiControl::StartWaitingForOnboardCompleteMessage() {
  ADK_LOG_ERROR("Wi-Fi-Control::StartWaitingForOnboardCompleteMessage()");
  data_->PerformWaitingForOnboardCompleteMessage();
}

//  Wi-Fi re-connect with home-AP
void WiFiControl::StaReconnectToHomeAP() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StaReconnectToHomeAP()");
  data_->StartStaReconnectToHomeAP();
}

//  Enable Wi-Fi in MLAN AP mode
void WiFiControl::EnableInMlanApMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::EnableInMlanApMode()");

  if (!data_->PerformSetMlanApMode()) {
    ADK_LOG_ERROR("Wi-Fi-Control::EnableInMlanApMode() failed");

    // Send failed indications in ADK-IPC bus
    data_->SendMlanStartLeadStatus(false, "" , "");
  }
}

//  Enable Wi-Fi in STA (home-AP)+MLAN AP mode
void WiFiControl::EnableInHomeStaMlanApMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::EnableInHomeStaMlanApMode()");

  if (!data_->PerformSetHomeStaMlanApMode()) {
    ADK_LOG_ERROR("Wi-Fi-Control::EnableInHomeStaMlanApMode() failed");

    // Send failed indications in ADK-IPC bus
    data_->SendMlanStartLeadStatus(false, "", "");
  }
}

//  Enable Wi-Fi STA mode (for connection to home-AP)
void WiFiControl::EnableInHomeStaMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::EnableInHomeStaMode()");

  if (!data_->PerformSetHomeStaMode()) {
    ADK_LOG_ERROR("Wi-Fi-Control::PerformSetHomeStaMode() failed");
  }
}

//  Stop Wi-Fi activities
void WiFiControl::Stop() {
  ADK_LOG_DEBUG("Wi-Fi-Control::Stop()");

  // We don't have a Wi-Fi driver interface for just stopping Wi-Fi activities with
  // the Wi-Fi still kept enabled, so disable the Wi-Fi for now
  if (!data_->PerformDisable()) {
    ADK_LOG_ERROR("Wi-Fi-Control::stopping/disabling Wi-Fi failed");
    data_->SendDisableStatus(false);
  }
}

// Set the enable bit in configuration
void WiFiControl::SetEnableInConfig() {
  ADK_LOG_DEBUG("Wi-Fi-Control::SetEnableInConfig()");

  if (!data_->SetEnableState(true)) {
    ADK_LOG_ERROR("Wi-Fi-Control::setting Wi-Fi enable config failed");
    return;
  }

  data_->SendEnableStatus(true);
}

//  Enable Wi-Fi in MLAN Slave mode for connecting to MLAN AP
void WiFiControl::EnableInMlanStaMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::EnableInMlanStaMode()");

  if (!data_->PerformSetMlanSlaveMode()) {
    ADK_LOG_ERROR("Wi-Fi-Control::starting re-connect to MLAN AP failed");
    data_->SendMlanStartSlaveStatus(false, "");
  }
}

//  Wi-Fi MLAN Onboarding start
void WiFiControl::StartMlanOnboarding() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StartMlanOnboarding()");

  std::string ssid;
  std::string passphrase;

  //  Get the SSID and password from the input ADK-IPC message
  if (data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanonboardingstart().has_ssid()) {
      ssid = data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanonboardingstart().ssid();
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi connect invalid para: no SSID");
    data_->SendMlanStartOnboardingStatus(false, ssid);
    return;
  }

  if (data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanonboardingstart().has_password()) {
    passphrase = data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanonboardingstart().password();
  }

  //  Wait for connection with the input AP
  if (!data_->StaConnect(ssid, passphrase, false)) {
    data_->SendMlanStartOnboardingStatus(false, ssid);
  }
}

// Send Wi-Fi MLAN Onboarding status
void WiFiControl::SendMlanOnboardingStatus() {
  ADK_LOG_DEBUG("Wi-Fi-Control::SendMlanOnboardingStatus()");

  std::string ssid;
  data_->GetStaConnectionSSID(&ssid);

  data_->SendMlanStartOnboardingStatus(true, ssid);
}

// Send the current Wi-Fi Enabled status
void WiFiControl::SendEnabledStatus(bool enabled) {
  data_->SendEnableStatus(enabled);
}

// Send the current Wi-Fi Disabled status
void WiFiControl::SendDisabledStatus(bool disabled) {
  data_->SendDisableStatus(disabled);
}


//  Wi-Fi Stop MLAN Onboarding
void WiFiControl::StopMlanOnboarding() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StopMlanOnboarding()");

  // Send stop onboarding response
  data_->SendMlanStopOnboardingStatus(true);
}

// Wi-Fi MLAN start slave
void WiFiControl::StartMlanSlave() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StartMlanSlave()");

  std::string ssid;
  std::string passphrase;

  // Extact the MLAN ssid and password from the input ADK IPC message
  if (data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanstartslave().has_ssid()) {
    ssid = data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanstartslave().ssid();
  } else {
    // We cannot proceed with the MLAN connection if there's no input SSID!
    data_->SendMlanStartSlaveStatus(false, ssid);
    return;
  }

  if (data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanstartslave().has_password()) {
    passphrase = data_->received_command_.payload_adk_ipc.connectivity_wifi_mlanstartslave().password();
  }

  //  Update the MLAN state
  data_->SetMlanState(MLAN_NOT_ACTIVE);

  // Update the WPA supplicant conf file with the MLAN AP SSID/password
  data_->UpdateWPASupplicantConfFile(ssid,passphrase);

  // Take a back-up of the wpa_supplicant conf file for the MLAN connection
  data_->BackupMlanWPASupplicantConf();

  if (!data_->PerformSetMlanSlaveMode()) {
    ADK_LOG_ERROR("Wi-Fi-Control::starting connect to MLAN AP failed");
    data_->SendMlanStartSlaveStatus(false, ssid);
  }
}

//  Wi-Fi STA (home-STA)+ MLAN lead start
void WiFiControl::StartHomeStaMlanLead() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StartHomeStaMlanLead()");

  std::string ssid, password;

  if (data_->PerformSetHomeStaMlanApMode() && data_->GetMlanAPCredentials(&ssid, &password) ) {
    data_->SendMlanStartLeadStatus(true, ssid, password);

    // Set the MLAN lead state
    data_->SetMlanState(MLAN_LEAD);
  } else {
    data_->SendMlanStartLeadStatus(false, "", "");
  }
}

//  Wi-Fi MLAN lead start
void WiFiControl::StartMlanLead() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StartMlanLead()");

  std::string ssid, password;

  if (data_->PerformSetMlanApMode() && data_->GetMlanAPCredentials(&ssid, &password) ) {
    data_->SendMlanStartLeadStatus(true, ssid, password);

    // Set the MLAN lead state
    data_->SetMlanState(MLAN_LEAD);
  } else {
    data_->SendMlanStartLeadStatus(false, "", "");
  }
}

//  Action Wi-Fi MLAN  stop
void WiFiControl::StopMlan() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StopMlan()");

  //  Reset MLAN lead to default value
  data_->SetMlanState(MLAN_NOT_ACTIVE);

  //  Return the stop status
  data_->SendMlanStopStatus(true);
}

//  Enter home-AP onboarded mode
void WiFiControl::SetOnboardedMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::SetOnboardedMode()");

  // Set the onboarded state
  data_->SetOnboardState(true);
  std::string ssid;
  data_->GetStaConnectionSSID(&ssid);
  data_->SendOnboardStatus(true, ssid);

  //  STA mode connected to a AP.
  //  Take a backup of the current wpa_supplicant.conf file as it's required
  //  to revert to home-AP if a new connection fails later on
  if (!data_->BackupHomeWPASupplicantConf()) {
    ADK_LOG_ERROR( "Wi-Fi-Control::backup wpa_supplicant conf failed");
  }
}

//  Wi-Fi MLAN slave connected
void WiFiControl::SetMlanSlaveConnected() {
  ADK_LOG_DEBUG("Wi-Fi-Control::SetMlanSlaveConnected()");

   //  Update the MLAN  state
  data_->SetMlanState(MLAN_SLAVE);

  std::string ssid;
  data_->GetStaConnectionSSID(&ssid);
  data_->SendMlanStartSlaveStatus(true, ssid);
}

//  Wi-Fi MLAN reset state
void WiFiControl::ResetMlanState() {
  ADK_LOG_DEBUG("Wi-Fi-Control::ResetMlanState()");

   //  Reset MLAN state
  data_->SetMlanState(MLAN_NOT_ACTIVE);
}

//  Send network info
void WiFiControl::SendNetworkInfo() {
  ADK_LOG_DEBUG("Wi-Fi-Control::SendNetworkInfo()");

  data_->SendNetworkInfo();
}

// Send unhandled ADK-IPC message
void WiFiControl::SendErrorMessage(int unhandled_event) {
  ADK_LOG_DEBUG("Wi-Fi-Control::SendErrorMessage() for unhandled event:%d", unhandled_event);

  // Send ADK-IPC message responses (with status = fail) for the following unhandled events
  switch(unhandled_event) {
    case adk::connectivity::nsm::kNSMWiFiEnableEventID: {
      data_->SendEnableStatus(false);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiDisableEventID: {
      data_->SendDisableStatus(false);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiScanEventID: {
      std::vector<ScanInfo> in_scan_list;
      data_->SendScanResults(false, in_scan_list);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiConnectEventID: {
      data_->SendConnectStatus(false, "");
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiOnboardEventID: {
      data_->SendOnboardingStatus(false);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiMlanOnboardingStartEventID: {
      data_->SendMlanStartOnboardingStatus(false, "");
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiMlanOnboardingStopEventID: {
      data_->SendMlanStopOnboardingStatus(false);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiMlanStartLeadEventID: {
      data_->SendMlanStartLeadStatus(false, "", "");
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiMlanStopEventID: {
      data_->SendMlanStopStatus(false);
      break;
    }

    case adk::connectivity::nsm::kNSMWiFiMlanStartSlaveEventID: {
      data_->SendMlanStartSlaveStatus(false, "");
      break;
    }

    default: {
      ADK_LOG_INFO("Wi-Fi-Control::no ADK-IPC message sent for the unhandled event:%d", unhandled_event);
      break;
    }

  }

}


// Return Wi-Fi enabled state
bool WiFiControl::IsEnabled() {
  return data_->Enabled();
}

// Return Wi-Fi onboarded state
bool WiFiControl::IsOnboarded() {
  return data_->Onboarded();
}

// Get MLAN Lead state
bool WiFiControl::IsMlanLeadEnabled() {
  return data_->MlanLeadEnabled();
}

// Get MLAN slave state
bool WiFiControl::IsMlanSlaveEnabled() {
  return data_->MlanSlaveEnabled();
}

// Get Ethernet connection satus
bool WiFiControl::IsEthernetUp() {
  return (data_->ethernet_connected_);
}

// Get Wi-Fi STA mode connection satus
bool WiFiControl::IsWiFiStaConnected() {
  return (data_->sta_connected_);
}

// Perform Wi-Fi disable
bool WiFiControl::Private::PerformDisable() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformDisable()");

  //  Reset NSM timers
  ResetControlTimer();

  if (!qcmap_control_->enableWlan(false)) {
    ADK_LOG_ERROR("Wi-Fi-Control::disable failed");

    // Try again once more
    if (!qcmap_control_->enableWlan(false)) {
      ADK_LOG_ERROR("Wi-Fi-Control::disable retry failed");
      return false;
    }
  }
  ADK_LOG_DEBUG("Wi-Fi-Control::Wi-Fi disabled");
  return true;
}

//  Start Wi-Fi onboarding
bool WiFiControl::Private::PerformOnboarding() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformOnboarding()");

  //  Reset NSM timers
  ResetControlTimer();

  //  Set Onboard AP + STA configuration
  if (UpdateOnboardAPConfig(true)) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set Onboard AP config successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set Onboard AP config fail");
    return false;
  }

  //  Set AP(Soft-AP)+STA mode
  // if (qcmap_control_->setWlanModeApSTA()) {
    // ADK_LOG_DEBUG("Wi-Fi-Control::Set Onboard-AP+STA mode successful");
  // } else {
    // ADK_LOG_ERROR("Wi-Fi-Control::Set Onboard-AP+STA mode failed");
    // return false;
  // }

  //  Remove wpa_supplicant entries so that STA mode will not get
  //  connected when the device is going to be onboarded with another AP
  if (!UpdateWPASupplicantConfFile("","")) {
    ADK_LOG_ERROR("Wi-Fi-Control::Update wpa_supplicant conf failed");
    return false;
  }
  // Reset the connected flag
   sta_connected_ = false;

  //  Wi-Fi modes changed, Activate Wi-Fi to get it effective
  if (!Activate()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi Activate failed");
    return false;
  }

  // Update country code for the Onboard AP
  if (!UpdateCountryCode(adk::Connectivity::keys::wifi::kWiFiOnboardAPCountryCode)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Update country code failed for onboard AP");
    return false;
  }

  return true;
}

//  Wi-Fi complete onboarding timeout
void WiFiControl::Private::OnboardingCompleteTimeout() {
  ADK_LOG_ERROR("Wi-Fi-Control::OnboardingCompleteTimeout()");
  ResetControlTimer();

  // Didn't receive a complete onboarding message from peer device,
  // so send a complete onboarding complete message to self on timeout
  // to compete the onboarding process if the STA mode is still connected.

  if (sta_connected_) {
    ConnectivityCommand connectivity_command;

    connectivity_command.command = CONNECTIVITY_WIFI_COMPLETE_ONBOARDING;
    connectivity_queue->add(connectivity_command);
  }
}

//  Wi-Fi start timer waiting for onboarding completion
void WiFiControl::Private::PerformWaitingForOnboardCompleteMessage() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformWaitingForOnboardCompleteMessage()");

  int timeout_sec = 0;

  //  Onboarding complete timeout - how long to wait for mobile-app/peer device
  //  to complete the onboarding process
  // connectivity_wifi_config_->Read(&timeout_sec,
      // adk::Connectivity::keys::wifi::kWiFiOnboardingCompleteTimeout);

  if (timeout_sec) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Wait %d sec for onboard complete message",timeout_sec);
    timer_task_ = Timer::create()->createTask(
                      [this]() { OnboardingCompleteTimeout(); },
                       std::chrono::seconds{timeout_sec});

  } else {
  //  No need to wait for onboard complete message, so invoke the timeout function now itself
    OnboardingCompleteTimeout();
  }
}

//  Wi-Fi reset Wi-Fi control timer
void WiFiControl::Private::ResetControlTimer() {
  ADK_LOG_ERROR("Wi-Fi-Control::ResetControlTimer()");
  if (timer_task_) {
    timer_task_->cancel(true);
    timer_task_.reset();
  }
}

//  Set Wi-Fi STA (for connecting to home-AP) + MLAN AP mode
bool WiFiControl::Private::PerformSetHomeStaMlanApMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformSetHomeStaMlanApMode()");

  //  Reset NSM timers
  ResetControlTimer();

  // Restore the home-AP wpa_supplicant conf file if onboarded
  bool onboarded;

  if (!GetOnboardState(&onboarded)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Get onboarded config failed");
    return false;
  }
  if (onboarded) {
    RestoreHomeWPASupplicantConf();
  }

  //  Set MLAN AP + STA configuration
  if (UpdateMlanAPConfig(true)) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set MLAN AP config successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set MLAN AP config failed");
    return false;
  }

  //  Set STA+AP mode
  if (qcmap_control_->setWlanModeApSTA()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set STA+AP mode successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set STA+AP failed");
    return false;
  }

  //  Set LAN configuration for the MLAN AP
  if (!UpdateAPLANConfiguration()) {
    ADK_LOG_ERROR("Wi-Fi-Control::AP LAN configuration failed");
    return false;
  }

  //  Wi-Fi modes changed, Activate Wi-Fi to get it effective
  if (!Activate()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi Activate failed");;
    return false;
  }

  // Set the MLAN AP Tx power
  // MLAN AP would be in wlan1 interface if both STA+AP modes are enabled
  // TODO: the interface will be read from ADK database file or from a QCMAP API
  // when we add dual radio support
  if (!UpdateMlanAPTxPower(kMlanAPWithHomeStaInterface)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Set MLAN AP Tx power fail");
    return false;
  }

  // Update country code for the MLAN AP
  if (!UpdateCountryCode(adk::Connectivity::keys::wifi::kWiFiMlanAPCountryCode)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Update country code failed for MLAN AP");
    return false;
  }

  // Re-connect with home-AP with STA mode is enabled{
  StaConnecting();

  return true;
}

//  Set Wi-Fi MLAN AP mode
bool WiFiControl::Private::PerformSetMlanApMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformSetMlanApMode()");

  //  Reset NSM timers
  ResetControlTimer();

  //  Set MLAN AP configuration
  if (UpdateMlanAPConfig(false)) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set MLAN AP config successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set MLAN AP config failed");
    return false;
  }

  //  Set AP mode
  if (qcmap_control_->setWlanModeAp()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set AP mode successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set AP failed");
    return false;
  }

  //  Set LAN configuration for the MLAN AP
  if (!UpdateAPLANConfiguration()) {
    ADK_LOG_ERROR("Wi-Fi-Control::AP LAN configuration failed");
    return false;
  }

  //  Wi-Fi modes changed, Activate Wi-Fi to get it effective
  if (!Activate()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi Activate failed");;
    return false;
  }

  // Set the MLAN AP Tx power
  // MLAN AP would be in wlan0 interface if only AP mode is enabled
  // TODO: the interface will be read from ADK database file or from a QCMAP API
  // when we add dual radio support
  if (!UpdateMlanAPTxPower(kMlanAPWithoutHomeStaInterface)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Set MLAN AP Tx power fail");
    return false;
  }

  // Update country code for the MLAN AP
  // TODO: the wpa_cli currently fails if the STA mode is not active
  if (!UpdateCountryCode(adk::Connectivity::keys::wifi::kWiFiMlanAPCountryCode)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Update country code failed for MLAN AP");
  }

  return true;
}

//  Set Wi-Fi STA mode for Home-AP connection
bool WiFiControl::Private::PerformSetHomeStaMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformSetHomeStaMode()");

  bool onboarded;

  //  Reset NSM timers
  ResetControlTimer();

  if (!GetOnboardState(&onboarded)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Get onboarded config failed");
    return false;
  }

  // Restore the home-AP wpa_supplicant conf file if onboarded before
  if (onboarded) {
    RestoreHomeWPASupplicantConf();
  }

  //  Set STA mode for connecting to home AP
  if (qcmap_control_->setWlanModeSta()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set STA mode successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set STA mode failed");
    return false;
  }

  //  Wi-Fi modes changed, Activate Wi-Fi to get it effective
  if (!Activate()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi Activate failed");;
    return false;
  }

  // Re-connect with home-AP
  StaConnecting();

  return true;
}

//  Set MLAN Slave mode
bool WiFiControl::Private::PerformSetMlanSlaveMode() {
  ADK_LOG_DEBUG("Wi-Fi-Control::PerformSetMlanSlaveMode()");

  //  Reset NSM timers
  ResetControlTimer();

  // Restore the MLAN WPA supplicant conf file
  if (!RestoreMlanWPASupplicantConf()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Restore MLAN-AP wpa_suppplicant failed");
    return false;
  }

  //  Set STA mode for connecting to home AP
  if (qcmap_control_->setWlanModeSta()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Set STA mode successful");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Set STA mode failed");
    return false;
  }

  //  Wi-Fi modes changed, Activate Wi-Fi to get it effective
  if (!Activate()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi Activate failed");;
    return false;
  }

  // Re-connect with home-AP
  StaConnecting();

  return true;
}

//  Get Wi-Fi enabled state
bool WiFiControl::Private::GetEnableState(bool *state) {
  assert(state != nullptr);

  if (!connectivity_wifi_config_->Read(
          state, adk::Connectivity::keys::wifi::kWiFiEnable)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi enable state read failed");
    return false;
  }
  return true;
}

//  Set Wi-Fi enabled state
bool WiFiControl::Private::SetEnableState(const bool state) {
  if (!connectivity_wifi_config_->Write(
          state, adk::Connectivity::keys::wifi::kWiFiEnable)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi enabled state write failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi onboarded state from connectivity.states db file
bool WiFiControl::Private::GetOnboardState(bool *state) {
  assert(state != nullptr);

  if (!connectivity_states_config_->Read(state,
      adk::Connectivity::keys::wifi::kWiFiOnboarded)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi onboarded read failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN lead state from connectivity.states db file
bool WiFiControl::Private::GetMlanState(MlanState *state) {
  assert(state != nullptr);
  if (!connectivity_states_config_->Read(state,
      adk::Connectivity::keys::wifi::kWiFiMlanState)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi get MLAN state read failed");
    return false;
  }
  return true;
}

//  Set Wi-Fi onboarded state in connectivity.states db file
bool WiFiControl::Private::SetOnboardState(const bool state) {
  if (!connectivity_states_config_->Write(state,
      adk::Connectivity::keys::wifi::kWiFiOnboarded)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi onboarded state write failed");
    return false;
  }
  return true;
}

//  Set Wi-Fi MLAN state in connectivity.states db file
bool WiFiControl::Private::SetMlanState(const MlanState state) {
  if (!connectivity_states_config_->Write(state,
      adk::Connectivity::keys::wifi::kWiFiMlanState)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi set MLAN state write failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN AP gateway IP address from connectivity.wifi db file
bool WiFiControl::Private::GetMlanAPGatewayIPAddr(std::string *ap_gateway_addr) {
  assert(ap_gateway_addr != nullptr);

  if (!connectivity_wifi_config_->Read(
          ap_gateway_addr,
          adk::Connectivity::keys::wifi::kWiFiMlanAPGatewayAddr)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi get MLAN AP gateway address read failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN AP netmask from connectivity.wifi db file
bool WiFiControl::Private::GetMlanAPNetmask(std::string *ap_netmask) {
  assert(ap_netmask != nullptr);

  if (!connectivity_wifi_config_->Read(
      ap_netmask,
          adk::Connectivity::keys::wifi::kWiFiMlanAPNetmask)) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Wi-Fi get MLAN AP gateway netmask read failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN AP DHCP start address from connectivity.wifi db file
bool WiFiControl::Private::GetMlanDHCPStartAddr(std::string *ap_dhcp_start_addr) {
  assert(ap_dhcp_start_addr != nullptr);

  if (!connectivity_wifi_config_->Read(
      ap_dhcp_start_addr,
          adk::Connectivity::keys::wifi::kWiFiMlanAPDHCPStartAddress)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi get MLAN AP DHCP start address read failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN AP DHCP end address from connectivity.wifi db file
bool WiFiControl::Private::GetMlanDHCPEndAddr(std::string *ap_dhcp_end_addr) {
  assert(ap_dhcp_end_addr != nullptr);

  if (!connectivity_wifi_config_->Read(
      ap_dhcp_end_addr,
          adk::Connectivity::keys::wifi::kWiFiMlanAPDHCPEndAddress)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi get MLAN AP DHCP end address read failed");
    return false;
  }
  return true;
}

//  Get Wi-Fi MLAN AP DHCP lease time from connectivity.wifi db file
bool WiFiControl::Private::GetMlanDHCPLeaseTime(int *ap_dhcp_lease_time) {
  assert(ap_dhcp_lease_time != nullptr);

  if (!connectivity_wifi_config_->Read(
      ap_dhcp_lease_time,
          adk::Connectivity::keys::wifi::kWiFiMlanAPDHCPLeaseTime)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi get MLAN AP DHCP lease time read failed");
    return false;
  }
  return true;
}

// Update country code in Wi-Fi driver
bool WiFiControl::Private::UpdateCountryCode(const char *config_key) {
  std::string cmd;
  std::string key_value;

  assert(config_key != nullptr);

  ADK_LOG_DEBUG("Wi-Fi-Control::UpdateCountryCode() with key:%s", config_key);

  //  Read the key value for the input key
  if (!connectivity_wifi_config_->Read(&key_value, config_key)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to read key:%s", config_key );
    return false;
  }

  // Update country code using wpa_cli driver interface
  cmd = "wpa_cli";
  cmd += " DRIVER COUNTRY";
  cmd += " " + key_value;
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str() );
    return false;
  }

  return true;
}


bool WiFiControl::Private::UpdateOnboardAPConfig(const bool sta_enabled) {
  std::string onboard_key_value;
  std::string onboard_key_value_1;
  std::string onboard_key_value_mod;

  std::string cmd;

  ADK_LOG_DEBUG("Wi-Fi-Control::UpdateOnboardAPConfig()");

  cmd = "adk_wifi_config_update";
  if (sta_enabled) {
    cmd += " create_temp_sta_mode_hostapd_conf";
  } else {
    cmd += " create_temp_hostapd_conf";
  }

  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
    return false;
  }

  //  Set the Onboard soft-AP SSID by reading the config data_base
  //  onboard_ap_ssid_prefix + serial number
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPSSIDPrefix) &&
      device_config_->Read(&onboard_key_value_1,
                          adk::Connectivity::keys::wifi::kDeviceSerialNum)) {
    onboard_key_value_mod = onboard_key_value + "-" + onboard_key_value_1;
    EscapeForShell(onboard_key_value_mod);

    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " ssid_sta_mode_hostapd_conf";
    } else {
      cmd += " ssid_hostapd_conf";
    }
    cmd += " " + onboard_key_value_mod;
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Onboard AP SSID read config failed ");
    return false;
  }

#if UPDATE_WIFI_ADVANCED_PARAMS == 1

  //  Set Onboard soft-AP hw_mode
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPHWMode)) {

    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " hw_mode";
    cmd += " " + std::string(onboard_key_value);

    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Onboard AP hw_mode config failed");

    return false;
  }

  //  Set Onboard soft-AP channel
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPChannel)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " channel";
    cmd += " " + std::string(onboard_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Onboard AP channel config failed ");
    return false;
  }

  //  Set Onboard soft-AP ht_capab
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPHTCapability)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " ht_capab";
    cmd += " " + std::string(onboard_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }

  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Onboard AP ht_capab config failed ");
    return false;
  }

  //  Set Onboard soft-AP country code
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPCountryCode)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " country_code";
    cmd += " " + std::string(onboard_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Onboard AP country code config failed");
    return false;
  }

  //  Set Onboard AP broadcast SSID
  if (connectivity_wifi_config_->Read(
          &onboard_key_value,
          adk::Connectivity::keys::wifi::kWiFiOnboardAPIgnoreBroadcastSSID)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " ignore_broadcast_ssid";
    cmd += " " + std::string(onboard_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::ignore_broadcast_ssid config failed");
    return false;
  }

#endif /* UPDATE_WIFI_ADVANCED_PARAMS */

  cmd = "adk_wifi_config_update";
  if (sta_enabled) {
    cmd += " commit_sta_mode_hostapd_conf";
  } else {
    cmd += " commit_hostapd_conf";
  }

  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
    return false;
  }

  return true;
}

//  Get MLAN AP credentials
bool WiFiControl::Private::GetMlanAPCredentials(std::string *ret_ssid,
                                                std::string *ret_password) {
  std::string serial_num;
  std::string mlan_ap_ssid_prefix;

  assert(ret_ssid != nullptr);
  assert(ret_password != nullptr);

  ADK_LOG_DEBUG("Wi-Fi-Control::GetMlanAPCredentials()");

  //  Currently the MLAN AP SSID and password are read from wifi configuration
  //  database
  //  The SSID contains the serial number of the device
  if (connectivity_wifi_config_->Read(
          &mlan_ap_ssid_prefix,
          adk::Connectivity::keys::wifi::kWiFiMlanAPSSIDPrefix) &&
      device_config_->Read(&serial_num,
                          adk::Connectivity::keys::wifi::kDeviceSerialNum) &&
      connectivity_wifi_config_->Read(
          ret_password, adk::Connectivity::keys::wifi::kWiFiMlanAPPassword)) {
    *ret_ssid = mlan_ap_ssid_prefix + "-" + serial_num;
    return true;
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP SSID config read fail");

    return false;
  }
}

//  Update MLAN AP Config parameters
bool WiFiControl::Private::UpdateMlanAPConfig(bool sta_enabled) {
  std::string  cmd;
  std::string  mlan_key_value;
  std::string  mlan_key_value_1;

  ADK_LOG_DEBUG("Wi-Fi-Control::Backup UpdateMlanAPConfig()");

  cmd = "adk_wifi_config_update";
  if (sta_enabled) {
    cmd += " create_temp_sta_mode_hostapd_conf";
  } else {
    cmd += " create_temp_hostapd_conf";
  }

  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
    return false;
  }

  //  Set MLAN AP SSID and password
  if (GetMlanAPCredentials(&mlan_key_value, &mlan_key_value_1)) {
    EscapeForShell(mlan_key_value);
    EscapeForShell(mlan_key_value_1);
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " ssid_and_password_sta_mode_hostapd_conf";
    } else {
      cmd += " ssid_and_password_hostapd_conf";
    }
    cmd += " " + mlan_key_value;
    cmd += " " + mlan_key_value_1;
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP get credentials failed");
    return false;
  }

#if UPDATE_WIFI_ADVANCED_PARAMS == 1

  //  Set MLAN AP channel number
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPChannel)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " channel";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP channel read failed ");
    return false;
  }

  //  Set MLAN beacon interval
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPBeaconInterval)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " beacon_int";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP beacon config read failed");
    return false;
  }

  //  Set MLAN WMM_enabled flag
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPWMMEnable)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " wmm_enabled";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP WMM config read failed ");
    return false;
  }

  //  Set MLAN DTIM period
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPDTIMPeriod)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " dtim_period";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP DTIM config read failed");
    return false;
  }

  //  Set MLAN AP hw_mode
  if (connectivity_wifi_config_->Read(
          &mlan_key_value, adk::Connectivity::keys::wifi::kWiFiMlanAPHWMode)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " hw_mode";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP hw_mode config failed ");
    return false;
  }

  //  Set MLAN AP channel
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPChannel)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " channel";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP channel config failed ");
    return false;
  }

  //  Set MLAN AP ht_capab
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPHTCapability)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " ht_capab";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ht_capab config failed ");
    return false;
  }

  //  Set MLAN country code
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPCountryCode)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " country_code";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP country config read fail");
    return false;
  }

  //  Set MLAN AP ieee80211d
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAP80211dEnable)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " ieee80211d";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ieee80211d config failed");
    return false;
  }

  //  Set MLAN AP ieee80211h
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAP80211hEnable)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " ieee80211h";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ieee80211h config failed ");
    return false;
  }

  //  Set MLAN AP ieee80211ac
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAP80211acEnable)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " ieee80211ac";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ieee80211ac config failed");
    return false;
  }

  //  Set MLAN AP ieee80211n
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAP80211nEnable)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " ieee80211n";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ieee80211n config failed ");
    return false;
  }

  //  Set MLAN AP max_num_sta
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPMaxNumStations)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }

    cmd += " max_num_sta";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP max_num_sta config failed ");
    return false;
  }

  //  Set MLAN AP broadcast SSID
  if (connectivity_wifi_config_->Read(
          &mlan_key_value,
          adk::Connectivity::keys::wifi::kWiFiMlanAPIgnoreBroadcastSSID)) {
    cmd = "adk_wifi_config_update";
    if (sta_enabled) {
      cmd += " param_sta_mode_hostapd_conf";
    } else {
      cmd += " param_hostapd_conf";
    }
    cmd += " ignore_broadcast_ssid";
    cmd += " " + std::string(mlan_key_value);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP ignore_broadcast_ssid config failed");
    return false;
  }

#endif /* UPDATE_WIFI_ADVANCED_PARAMS */

  cmd = "adk_wifi_config_update";
  if (sta_enabled) {
    cmd += " commit_sta_mode_hostapd_conf";
  } else {
    cmd += " commit_hostapd_conf";
  }

  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
    return false;
  }

  return true;
}

//  Set Wi-Fi MLAN AP Tx power
bool WiFiControl::Private::UpdateMlanAPTxPower(const std::string &wlan_interface) {
  std::string cmd;
  std::string mlan_ap_tx_power;

  ADK_LOG_DEBUG("Wi-Fi-Control::UpdateMlanAPTxPower() interface: %s", wlan_interface.c_str());

  //  Set MLAN AP Tx power once the Wi-Fi activated
  //  TBD: can this be done before enabling the Wi-Fi?
  if (connectivity_wifi_config_->Read(
          &mlan_ap_tx_power,
          adk::Connectivity::keys::wifi::kWiFiMlanAPMaxTxPower)) {
    std::string cmd = "iwpriv";
    cmd += " " + wlan_interface;
    cmd += " setTxPower";
    cmd += " " + std::string(mlan_ap_tx_power);
    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
      return false;
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::MLAN AP tx power failed ");
    return false;
  }
  return true;
}

//  Send Wi-Fi connect status
void WiFiControl::Private::SendConnectStatus(const bool status,
                                             const std::string &ssid) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_connected();

  if (status) {
    connected_msg->set_ssid(ssid);
    connected_msg->set_freq(GetStaConnectedFrequency());
    connected_msg->set_status("success");
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi connect status: %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_connected()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_connected() message not Initialised");
  }
}

//  Send Wi-Fi connecting status
void WiFiControl::Private::SendConnectingStatus(const std::string &ssid) {
  AdkMessage send_message;
  auto connecting_msg = send_message.mutable_connectivity_wifi_connecting();

  ADK_LOG_DEBUG("Wi-Fi-Control::SendConnectingStatus() ssid: %s", ssid.c_str());

  if (!ssid.empty()) {
    connecting_msg->set_ssid(ssid);
  }
  //  Creation of mutable object initializes the AdkMessage object
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_connecting()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_connecting() message not Initialised");
  }
}

//  Send Wi-Fi MLAN Start onboarding status
void WiFiControl::Private::SendMlanStartOnboardingStatus(
    const bool status, const std::string &ssid) {
  AdkMessage send_message;
  auto connected_msg =
      send_message.mutable_connectivity_wifi_mlanonboardingstarted();

  if (status) {
    connected_msg->set_ssid(ssid);
    connected_msg->set_status("success");
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG(
      "Wi-Fi-Control::Sending Wi-Fi MLAN onboarding starting status: %d",
      status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanonboardingstarted()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanonboardingstarted() message not Initialised");
  }
}

//  Send Wi-Fi MLAN Stop onboarding status
void WiFiControl::Private::SendMlanStopOnboardingStatus(const bool status) {
  AdkMessage send_message;
  auto connected_msg =
      send_message.mutable_connectivity_wifi_mlanonboardingstopped();

  if (status) {
    connected_msg->set_status("success");
  } 
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG(
      "Wi-Fi-Control::Sending Wi-Fi MLAN onboarding stopped status: %d",
      status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanonboardingstopped()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanonboardingstopped() message not Initialised");
  }
}

//  Send Wi-Fi MLAN start lead response
void WiFiControl::Private::SendMlanStartLeadStatus(
    const bool status, const std::string &mlan_ap_ssid,
    const std::string &mlan_ap_password) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_mlanleadstarted();

  if (status) {
    connected_msg->set_status("success");
    connected_msg->set_ssid(mlan_ap_ssid);
    connected_msg->set_password(mlan_ap_password);
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi MLAN lead start status:  %d",
                status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanleadstarted()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanleadstarted() message not Initialised");
  }
}

//  Send Wi-Fi MLAN stop lead response
void WiFiControl::Private::SendMlanStopStatus(const bool status) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_mlanstopped();

  if (status) {
    connected_msg->set_status("success");
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi MLAN stop status:  %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanstopped()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanstopped() message not Initialised");
  }
}

//  Send Wi-Fi MLAN get credentials response
void WiFiControl::Private::SendMlanAPGetCredentialsStatus(
    const bool status, const std::string &mlan_ap_ssid,
    const std::string &mlan_ap_password) {
  AdkMessage send_message;
  auto connected_msg =
      send_message.mutable_connectivity_wifi_mlanapgetcredentialsstatus();

  if (status) {
    connected_msg->set_status("success");
    connected_msg->set_ssid(mlan_ap_ssid);
    connected_msg->set_password(mlan_ap_password);
  } 
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi MLAN get credentials status:  %d",
                status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanapgetcredentialsstatus()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanapgetcredentialsstatus() message not Initialised");
  }
}

//  Send Wi-Fi MLAN start slave response
void WiFiControl::Private::SendMlanStartSlaveStatus(
    const bool status, const std::string &ssid) {
  AdkMessage send_message;
  auto connected_msg =
      send_message.mutable_connectivity_wifi_mlanslavestarted();

  if (status) {
    connected_msg->set_status("success");
    connected_msg->set_ssid(ssid);
  } 
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi MLAN slave start status: %d",
                status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_mlanslavestarted()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_mlanslavestarted() message not Initialised");
  }
}

//  Send Wi-Fi onboarding status
void WiFiControl::Private::SendOnboardingStatus(const bool status) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_onboarding();

  if (status) {
    connected_msg->set_status("success");
  } 
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi onboarding status: %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_onboarding()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_onboarding() message not Initialised");
  }
}

//  Send Wi-Fi onboard status
void WiFiControl::Private::SendOnboardStatus(const bool status, const std::string &ssid) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_onboarded();

  if (status) {
    connected_msg->set_status("success");
    connected_msg->set_ssid(ssid);
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi onboard status:  %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_onboarded()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_onboarded() message not Initialised");
  }
}

//  Send Wi-Fi enable status
void WiFiControl::Private::SendEnableStatus(const bool status) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_enabled();

  if (status) {
    connected_msg->set_status("success");
  } else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi enable status:  %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_enabled()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_enabled() message not Initialised");
  }
}

//  Send Wi-Fi disable status  //std::map<std::string, ipc::Service::AdkVariant>
//  map;
void WiFiControl::Private::SendDisableStatus(const bool status) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_disabled();

  if (status) {
    connected_msg->set_status("success");
  }
  else {
    connected_msg->set_status("fail");
  }

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi disable status:  %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_disabled()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_disabled() message not Initialised");
  }
}

//  Send Wi-Fi scan results
void WiFiControl::Private::SendScanResults(
    const bool status, const std::vector<ScanInfo> &in_scan_list) {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_wifi_scanresults();

  if (status) {
    std::vector<ScanInfo>::const_iterator it = in_scan_list.begin();
    std::vector<ScanInfo>::const_iterator end = in_scan_list.end();

    connected_msg->set_status("success");
    for (int index = 1; it != end; ++index, ++it) {
      auto scan_list = connected_msg->add_scan_result_list();
      boost::property_tree::ptree item;
      item.put("ssid", it->ssid);

      std::stringstream bssid;
      for (size_t index = 0; index < 6; index++) {
        if (index > 0) {
          bssid << ":";
        }
        bssid << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
              << (unsigned int)it->bssid[index];
      }
      scan_list->set_ssid(it->ssid);
      scan_list->set_bssid(bssid.str());
      scan_list->set_rssi(it->rssi);
      scan_list->set_caps(it->caps);
      scan_list->set_wpa(it->wpa);
    }

  } 
  else {
    connected_msg->set_status("fail");
  }
  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Wi-Fi scan status  %d", status);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
     ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_wifi_scanresults()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_wifi_scanresults() message not Initialised");
  }
}

//  Send Ethernet connected message
void WiFiControl::Private::SendEthernetConnected() {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_ethernet_connected();

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Ethernet connected message");

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_ethernet_connected()");
    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_ethernet_connected() message not Initialised");
  }
}

//  Send Ethernet disconnected message
void WiFiControl::Private::SendEthernetDisconnected() {
  AdkMessage send_message;
  auto connected_msg = send_message.mutable_connectivity_ethernet_disconnected();

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending Ethernet disconnected message");

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::SENDING MESSAGE:connectivity_ethernet_disconnected()");

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_ethernet_disconnected() message not Initialised");
  }
}

//  Send network info
void WiFiControl::Private::SendNetworkInfo() {
  AdkMessage send_message;
  std::string conn_interface;

  // Get the current NSM state
  adk::connectivity::nsm::NetworkState nsm_state = state_machine_->GetCurrentState();

  auto msg = send_message.mutable_connectivity_network_info();
  auto home_connection = msg->mutable_home_connection();
  auto mlan_connection = msg->mutable_mlan_connection();
  auto onboarding = msg->mutable_onboarding();
  auto bridge = msg->mutable_bridge();

  ADK_LOG_DEBUG("Wi-Fi-Control::Sending network info for NSM state:%d", nsm_state);

  //  Creation of mutable object initializes the AdkMessage obejct
  if (send_message.IsInitialized()) {
    //  Set the home connection parameters
    if(home_connection->IsInitialized()) {
      // Get Ethernet parameters if connected

      // Etherent connectivity is only allowed with the home-router
      if (ethernet_connected_ ||
          nsm_state == adk::connectivity::nsm::kNSMEthernetHome ||
          nsm_state == adk::connectivity::nsm::kNSMEthernetHome_WiFiMlanAp ) {
        home_connection->set_connectivity("ethernet");

        // Ethernet is always on "eth0" interface
        conn_interface = kEthernetInterface;
        home_connection->set_interface(conn_interface);

        // Set IP address
        home_connection->set_ip_address(GetIPAddress(conn_interface));

        //  Set MAC address
        home_connection->set_mac_address(GetMacAddress(conn_interface));
      } else if (sta_connected_ && (
                 nsm_state == adk::connectivity::nsm::kNSMWiFiOnboarding ||
                 nsm_state == adk::connectivity::nsm::kNSMWiFiHomeSta ||
                 nsm_state == adk::connectivity::nsm::kNSMWiFiHomeSta_WiFiMlanAp)) {
        home_connection->set_connectivity("wifi");

        // STA mode connection is always on "wlan0" interface
        conn_interface = kStaInterface;
        home_connection->set_interface(conn_interface);

        // Set IP address
        home_connection->set_ip_address(GetIPAddress(conn_interface));

        //  Set MAC address
        home_connection->set_mac_address(GetMacAddress(conn_interface));

        // Set SSID
        home_connection->set_ssid(GetStaConnectedSSID());

        // Set BSSID
        home_connection->set_bssid(GetStaConnectedBSSID());

        // Set frequency
        home_connection->set_frequency(GetStaConnectedFrequency());

        // Set RSSI
        home_connection->set_rssi(GetStaConnectedRSSI());

       // Set Encryption
       home_connection->set_encryption(GetStaConnectedEncryption());

      } else {
        home_connection->set_connectivity("disabled");
      }
    }

    //  Set the MLAN connection parameters
    if(mlan_connection->IsInitialized()) {
      if (nsm_state == adk::connectivity::nsm::kNSMWiFiHomeSta_WiFiMlanAp ||
          nsm_state == adk::connectivity::nsm::kNSMEthernetHome_WiFiMlanAp ||
          nsm_state == adk::connectivity::nsm::kNSMWiFiMlanAp ) {
        // MLAN AP is always over wifi
        mlan_connection->set_connectivity("wifi");
        mlan_connection->set_mode("AP");

        // The MLAN AP will be over "wlan1" interface if connected to home network over Wi-Fi
        // TODO: this might change when we have the dual radio support
        if (nsm_state == adk::connectivity::nsm::kNSMWiFiHomeSta_WiFiMlanAp) {
          conn_interface = kMlanAPWithHomeStaInterface;
        } else {
          conn_interface = kMlanAPWithoutHomeStaInterface;
        }
        mlan_connection->set_interface(conn_interface);

        // Set IP address
        mlan_connection->set_ip_address(GetIPAddress(conn_interface));

        // Set MAC address
        mlan_connection->set_mac_address(GetMacAddress(conn_interface));

        // Set AP mode SSID
        mlan_connection->set_ssid(GetAPModeSSID(conn_interface));

        // Set AP mode BSSID
        mlan_connection->set_bssid(GetAPModeBSSID(conn_interface));

        // Set AP mode frequency
        mlan_connection->set_frequency(GetAPModeFrequency(conn_interface));

      } else if (sta_connected_ && nsm_state == adk::connectivity::nsm::kNSMWiFiMlanSta ) {
        // MLAN STA is always over wifi!
        mlan_connection->set_connectivity("wifi");
        mlan_connection->set_mode("STA");

        // Mlan STA mode connection is always on "wlan0" interface
        conn_interface = kStaInterface;
        mlan_connection->set_interface(conn_interface);

        // Set IP address
        mlan_connection->set_ip_address(GetIPAddress(conn_interface));

        // Set MAC address
        mlan_connection->set_mac_address(GetMacAddress(conn_interface));

        // Set SSID
        mlan_connection->set_ssid(GetStaConnectedSSID());

        // Set BSSID
        mlan_connection->set_bssid(GetStaConnectedBSSID());

        // Set frequency
        mlan_connection->set_frequency(GetStaConnectedFrequency());

        // Set RSSI
        mlan_connection->set_rssi(GetStaConnectedRSSI());

       // Set Encryption
       mlan_connection->set_encryption(GetStaConnectedEncryption());

      } else {
        mlan_connection->set_connectivity("disabled");
      }
    }

    //  Set the bridge parameters
    if(bridge->IsInitialized()) {
      // Bridge is always over bridge0 interface
      conn_interface = "bridge0";
      bridge->set_interface(conn_interface);

      // Set IP address
      bridge->set_ip_address(GetIPAddress(conn_interface));
    }

    if(onboarding->IsInitialized()) {
      if (nsm_state == adk::connectivity::nsm::kNSMWiFiOnboarding) {
        // Onboarding is only over Wi-Fi for now
        onboarding->set_connectivity("wifi");

        // The Onboarding AP is always over "wlan1" interface
        conn_interface = kOnboardAPInterface;
        onboarding->set_interface(conn_interface);

        // Set IP address
        onboarding->set_ip_address(GetIPAddress(conn_interface));

        // Set MAC address
        onboarding->set_mac_address(GetMacAddress(conn_interface));

        // Set AP mode SSID
        onboarding->set_ssid(GetAPModeSSID(conn_interface));

        // Set AP mode BSSID
        onboarding->set_bssid(GetAPModeBSSID(conn_interface));

        // There's no encryption for Onboard AP
        onboarding->set_encryption("OPEN");

        // Set AP mode frequency
        onboarding->set_frequency(GetAPModeFrequency(conn_interface));
      }else {
        onboarding->set_connectivity("disabled");
      }
    }

    if (!message_service_->Send(send_message)) {
      ADK_LOG_ERROR("Wi-Fi-Control::ERROR: FAILED TO SEND NETWORK INFO MESSAGE");
    }
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::connectivity_network_info message not Initialised");
  }
}

//  Take a backup of the home AP wpa_supplicant conf file
bool WiFiControl::Private::BackupHomeWPASupplicantConf() {
  std::string cmd;

  ADK_LOG_DEBUG("Wi-Fi-Control::Backup homeAP wpa_supplicant");
  cmd.append("adk_wifi_config_update");
  cmd.append(" ").append("backup_home_wpa_supplicant_conf");
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
    return false;
  }
  return true;
}

//  Restore the backup home AP wpa_supplicant_conf file
bool WiFiControl::Private::RestoreHomeWPASupplicantConf() {
  std::string cmd;

  ADK_LOG_DEBUG("Wi-Fi-Control::Restore homeAP wpa_supplicant");
  cmd.append("adk_wifi_config_update");
  cmd.append(" ").append("restore_home_wpa_supplicant_conf");
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
    return false;
  }
  return true;
}

//  Take a backup of the MLAN wpa_supplicant conf file
bool WiFiControl::Private::BackupMlanWPASupplicantConf() {
  std::string cmd;

  ADK_LOG_DEBUG("Wi-Fi-Control::Backup MLAN wpa_supplicant");
  cmd.append("adk_wifi_config_update");
  cmd.append(" ").append("backup_mlan_wpa_supplicant_conf");
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
    return false;
  }
  return true;
}

//  Restore the backup MLAN wpa_supplicant_conf file
bool WiFiControl::Private::RestoreMlanWPASupplicantConf() {
  std::string cmd;

  ADK_LOG_DEBUG("Wi-Fi-Control::Restore MLAN wpa_supplicant");
  cmd.append("adk_wifi_config_update");
  cmd.append(" ").append("restore_mlan_wpa_supplicant_conf");
  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str());
    return false;
  }
  return true;
}

//  STA connect with the input AP
bool WiFiControl::Private::StaConnect(const std::string &ssid,
                                      const std::string &password,
                                      bool  connect_with_wps_router) {
  ADK_LOG_DEBUG("Wi-Fi-Control::STA connect AP: %s, password: %s, wps: %d",
                 ssid.c_str(), password.c_str(), connect_with_wps_router);

  bool all_fine = true;

  // Update the wpa_supplicant conf file with the input ssid and password
  if (!UpdateWPASupplicantConfFile(ssid, password)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Update wpa_supplicant conf failed");
    all_fine = false;
  }

  //  Now activate WPA supplicant
  if (all_fine && qcmap_control_->activateSupplicantConfig()) {
    ADK_LOG_DEBUG("Wi-Fi-Control::Wi-Fi activate supplicant success");
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi activate supplicant failed");
    all_fine = false;
  }

  // Invoke wpa_cli command to connect to a WPS enabled router
  if (all_fine && connect_with_wps_router) {
    std::string cmd;
    cmd.append("wpa_cli");
    cmd.append(" ").append("wps_pbc");

    if (!ExecuteSystem(cmd.c_str())) {
      ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str() );
      all_fine = false;
    }
  }

  //  Wait for the STA connection to AP
  if (all_fine) {
    StaConnecting();
  }
  return all_fine;
}

// STA Connection timedout event
void WiFiControl::Private::StaConnectedIndWaitTimeout() {
  ADK_LOG_DEBUG("Wi-Fi-Control::StaConnectedIndWaitTimeout()");
  ResetControlTimer();

  // Send connection failed status
  std::string ssid;
  GetStaConnectionSSID(&ssid);
  SendConnectStatus(false, ssid);


  // Send the connection timedout event to the state machine
  state_machine_->ProcessEvent(adk::connectivity::nsm::kNSMWiFiConnectionTimedoutEventID);
}

//  Reconnect with homeAP if it was onboarded before
bool WiFiControl::Private::StartStaReconnectToHomeAP() {
  bool onboarded = false;

  ADK_LOG_DEBUG("Wi-Fi-Control::StartStaReconnectToHomeAP()");

  GetOnboardState(&onboarded);
  // Try re-connecting with home-AP only if it was onboarded before.
  if(!onboarded) {
    ADK_LOG_ERROR("Wi-Fi-Control::not onboarded before");
    return false;
  }

  // Restore the home-AP WPA supplicant conf file
  if (!RestoreHomeWPASupplicantConf()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Restore home-AP wpa_suppplicant failed");
    return false;
  }

  if (!qcmap_control_->activateSupplicantConfig()) {
    ADK_LOG_ERROR("Wi-Fi-Control::activate wpa supplicant failed");
    return false;
  }

  // Send Wi-Fi connecting notification
  StaConnecting();
  return true;
}

//  Wait for STA connection establishment
void WiFiControl::Private::StaConnecting() {

  ADK_LOG_DEBUG("Wi-Fi-Control::StaConnecting()");

  int timeout_sec = 0;
  std::string ssid;
  GetStaConnectionSSID(&ssid);

  // Reset the connected flag while trying to establish a new connection
  sta_connected_ = false;

  //  Send connecting ADK-IPC message
  SendConnectingStatus(ssid);

  //  Read from config file
  connectivity_wifi_config_->Read(
      &timeout_sec,
      adk::Connectivity::keys::wifi::kWiFiSTAConnectTimeout);

  if (timeout_sec) {
    // Start timer to waiting for connection indication
    ADK_LOG_DEBUG("Wi-Fi-Control::wait:%d sec for connection indication", timeout_sec);
    timer_task_ = Timer::create()->createTask(
                      [this]() { StaConnectedIndWaitTimeout(); },
                       std::chrono::seconds{timeout_sec});
  } else {
    // This shouldn't really reach here as it would take few seconds to establish a Wi-Fi connection!
    // Invoke the timeout handler anyways
    StaConnectedIndWaitTimeout();
  }
}

//  Extract STA connect common parameters from input string
bool WiFiControl::Private::StaConnectGetCommonParams(AdkMessage input_data,
                                                     std::string *ssid,
                                                     std::string *passphrase) {
  if (!input_data.connectivity_wifi_connect().has_ssid()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi connect invalid para: no SSID");
    return false;
  }

  *ssid = input_data.connectivity_wifi_connect().ssid();

  //  Must have SSID value
  if (ssid->empty()) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi connect invalid SSID");
    return false;
  }

  if (input_data.connectivity_wifi_connect().has_password()) {
    *passphrase = input_data.connectivity_wifi_connect().password();
  }
  return true;
}

//  Get the current SSID in STA connection by reading wpa_supplicant.conf file
bool WiFiControl::Private::GetStaConnectionSSID(std::string *ssid_ptr) {
  std::regex   reg_exp("^\\s*ssid=\"(.*)\"");
  std::smatch  reg_exp_match;
  std::string  ssid_read = "";

  assert(ssid_ptr != nullptr);

  ADK_LOG_DEBUG("Wi-Fi Control::GetStaConnectionSSID()");

  std::ifstream wpa_file(adk::Connectivity::keys::wifi::kWpaSupplicantConfFile);
  std::string input_line;

  if (wpa_file) {
    while(std::getline(wpa_file, input_line)) {
      //  Search for the "ssid="xxx" in the line read
      if (std::regex_match(input_line,
                        reg_exp_match,
                        reg_exp)) {
        ssid_read = reg_exp_match[1];

        if (!ssid_read.empty()) {
          ADK_LOG_DEBUG("Wi-Fi-Control::ssid read: %s", ssid_read.c_str());

          //  Return the SSID read
          *ssid_ptr = ssid_read.c_str();
          wpa_file.close();
          return true;
        }
      }
    }
    wpa_file.close();
    ADK_LOG_ERROR("Wi-Fi-Control::ssid not in wpa_supplicant_conf file!");
    return false;

  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Failed to open file: %s",
                  adk::Connectivity::keys::wifi::kWpaSupplicantConfFile);
    return false;
  }
  return false;
}

//  Activate Wi-Fi after changing modes
bool WiFiControl::Private::Activate() {
  ADK_LOG_DEBUG("Wi-Fi-Control::Activate() - disable/enable Wi-Fi()");

  //  TO-DO: qcmap_control_->activateWlan() is not taking the
  //  mode changes (AP ---> AP+STA ---> STA, etc.),
  //  so disable and enable WLAN for now!

  // First disable Wi-Fi
  if (!qcmap_control_->enableWlan(false)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi disable during Activate failed");

   //Try disabling it once again
    if (!qcmap_control_->enableWlan(false)) {
      ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi re-disable during Activate failed");
      return false;
    }
  }

  // Now enable Wi-Fi
  if (!qcmap_control_->enableWlan(true)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi enable during Activate failed");

    // Wi-Fi activate failed, try enabling it once again
    if (!qcmap_control_->enableWlan(true)) {
      ADK_LOG_ERROR("Wi-Fi-Control::Wi-Fi re-enable during Activate failed");

      // Wi-Fi enable failed
      SendEnableStatus(false);
      return false;
    }
  }
    SetEnableState(true);
    SendEnableStatus(true);
  // Set Wi-Fi as enabled and send the message in ADK-IPC if Wi-Fi was disabled
  // by the user before
  bool wifi_enabled;
  if (!GetEnableState(&wifi_enabled)) {
    ADK_LOG_ERROR("Wi-Fi-Control::wifi_enabled read failed");
    return false;
  }

  if (!wifi_enabled) {
    SetEnableState(true);
    SendEnableStatus(true);
  }
  return true;
}

//  Update wpa_supplicant.conf file
bool WiFiControl::Private::UpdateWPASupplicantConfFile(const std::string &ssid,
                                                       const std::string &password) {

  ADK_LOG_DEBUG("Wi-Fi-Control::STA connect AP: %s, password: %s",
                   ssid.c_str(), password.c_str());
  std::string cmd;

 std::string escaped_ssid = ssid;
 std::string escaped_password = password;

  if(!escaped_ssid.empty()) {
      // Escape special characters (')
      EscapeForShell(escaped_ssid);
  }
  if(!escaped_password.empty()) {
      // Escape special characters (')
      EscapeForShell(escaped_password);
  }

  cmd.append("adk_wifi_wpa_supplicant_update");
  cmd.append(" ").append("-s");
  cmd.append(" ").append(escaped_ssid);
  if (!password.empty()) {
    cmd.append(" ").append("-p");
    cmd.append(" ").append(escaped_password);
  }

  if (!ExecuteSystem(cmd.c_str())) {
    ADK_LOG_ERROR("Wi-Fi-Control::executing command: %s  failed", cmd.c_str() );
    return false;
  }
  return true;
}

//  Update LAN by reading the MLAN AP configuration
bool WiFiControl::Private::UpdateAPLANConfiguration() {
  qcmap_msgr_lan_config_v01 lan_config;
  in_addr addr;
  std::string input_string;

  ADK_LOG_DEBUG("Wi-Fi-Control::UpdateAPLANConfiguration()");

  memset(&lan_config,0,sizeof(qcmap_msgr_lan_config_v01));

  // Gateway IP address
  memset(&addr,0,sizeof(in_addr));
  if (!GetMlanAPGatewayIPAddr(&input_string)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Gateway IP address read failed");
    return false;
  }

  if (inet_aton(input_string.c_str(), &addr)) {
    lan_config.gw_ip = ntohl(addr.s_addr);
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::Gateway IP address set failed");
    return false;
  }

  //  Netmask
  memset(&addr,0,sizeof(in_addr));
  if (!GetMlanAPNetmask(&input_string)) {
    ADK_LOG_ERROR("Wi-Fi-Control::netmask read failed");
    return false;
  }

  if (inet_aton(input_string.c_str(), &addr)) {
    lan_config.netmask = ntohl(addr.s_addr);
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::net mask set failed");
    return false;
  }

  // We only support DHCP LAN configuration
  lan_config.enable_dhcp = 1;

  //  DHCP start address
  memset(&addr,0,sizeof(in_addr));
  if (!GetMlanDHCPStartAddr(&input_string)) {
    ADK_LOG_ERROR("Wi-Fi-Control::DHCP start addr read failed");
    return false;
  }

  if (inet_aton(input_string.c_str(), &addr)) {
    lan_config.dhcp_config.dhcp_start_ip = ntohl(addr.s_addr);
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::DHCP start address read failed");
    return false;
  }

  //  DHCP end address
  memset(&addr,0,sizeof(in_addr));
  if (!GetMlanDHCPEndAddr(&input_string)) {
    ADK_LOG_ERROR("Wi-Fi-Control::DHCP end addr read failed");
    return false;
  }

  if (inet_aton(input_string.c_str(), &addr)) {
    lan_config.dhcp_config.dhcp_end_ip = ntohl(addr.s_addr);
  } else {
    ADK_LOG_ERROR("Wi-Fi-Control::DHCP end address read failed");
    return false;
  }

  int lease_time;
  if (!GetMlanDHCPLeaseTime(&lease_time)) {
    ADK_LOG_ERROR("Wi-Fi-Control::DHCP lease time read failed");
    return false;
  }

  //  DHCP lease time
  lan_config.dhcp_config.lease_time = lease_time;

  if (!qcmap_control_->setLANConfig(lan_config)) {
    ADK_LOG_ERROR("Wi-Fi-Control::setLANConfig() failed");
    return false;
  }

  //  activate LAN with the new DHCP configuration
  if (!qcmap_control_->activateLAN()) {
    ADK_LOG_ERROR("Wi-Fi-Control::activateLAN() failed");
    return false;
  }

  return true;
}

//  Escape the special\escape\non printable characters for system cmd execution
void WiFiControl::Private::EscapeForShell(std::string &user_input) {
#define ADD_QUOTE "'"

  std::string::size_type pos = 0;
  while ((pos = user_input.find("'", pos)) != std::string::npos) {
    user_input.replace(pos, 1, "'\\''");
    pos += 4;
  }
  user_input = ADD_QUOTE + user_input + ADD_QUOTE;
}

// Return Wi-Fi enabled state
bool WiFiControl::Private::Enabled() {
  bool wifi_enabled;

  if (!GetEnableState(&wifi_enabled)) {
    ADK_LOG_ERROR("Wi-Fi-Control::wifi_enabled read failed");
    return false;
  }
  return wifi_enabled;
}

// Return Wi-Fi onboarded state
bool WiFiControl::Private::Onboarded() {
  bool wifi_onboarded;

  if (!GetOnboardState(&wifi_onboarded)) {
    ADK_LOG_ERROR("Wi-Fi-Control::wifi_onboarded read failed");
    return false;
  }
  return wifi_onboarded;
}

// Get MLAN Lead enabled state
bool WiFiControl::Private::MlanLeadEnabled() {

  // Get the MLAN lead enabled event
  MlanState mlan_state = MLAN_NOT_ACTIVE;
  if (!GetMlanState(&mlan_state)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Get MLAN AP config failed");
    return false;
  }

  // Return true if MLAN lead is enabled
  return (mlan_state == MLAN_LEAD);
}

// Get MLAN slave enabled state
bool WiFiControl::Private::MlanSlaveEnabled() {

  // Get the MLAN lead enabled event
  MlanState mlan_state = MLAN_NOT_ACTIVE;
  if (!GetMlanState(&mlan_state)) {
    ADK_LOG_ERROR("Wi-Fi-Control::Get MLAN AP config failed");
    return false;
  }

  // Return true if MLAN slave is enabled
  return (mlan_state == MLAN_SLAVE);
}

//  Get IP address of the input interface
std::string WiFiControl::Private::GetIPAddress(const std::string &conn_interface) {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "ifconfig" for getting the IP address
  cmd = "ifconfig";
  cmd += " " + conn_interface;

  cmd += " | grep \"inet addr\" | cut -d ':' -f 2 | cut -d ' ' -f 1";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return cmd_return;
}

//  Get mac address of the input interface
std::string WiFiControl::Private::GetMacAddress(const std::string &conn_interface) {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "ifconfig" for getting the mac address
  cmd = "ifconfig";
  cmd += " " + conn_interface;

  if (conn_interface.compare(kEthernetInterface)) {
    cmd += " | grep HWaddr | cut -d ' ' -f 10";
  } else {
    cmd += " | grep HWaddr | cut -d ' ' -f 11";
  }
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return cmd_return;
}

//  Get connected SSID for the STA mode
std::string WiFiControl::Private::GetStaConnectedSSID() {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "wpa_cli" for getting the connected SSID
  cmd = "wpa_cli status | sed -n 's/^ssid=:*//p'";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return cmd_return;
}

//  Get connected BSSID for the STA mode
std::string WiFiControl::Private::GetStaConnectedBSSID() {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "wpa_cli" for getting the connected BSSID
  cmd = "wpa_cli status | sed -n 's/^bssid=:*//p'";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return (boost::to_upper_copy<std::string>(cmd_return));
}

//  Get STA connected frequency for STA mode
int WiFiControl::Private::GetStaConnectedFrequency() {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "wpa_cli" for getting the connected frequency
  cmd = "wpa_cli status | sed -n 's/^freq=:*//p'";
  cmd_return = ExecuteSystemReturn(cmd.c_str());

  if (!cmd_return.empty()) {
    return std::stoi(cmd_return,nullptr,0);
  } else {
    return 0;
  }
}

//  Get STA connected RSSI for STA mode
int WiFiControl::Private::GetStaConnectedRSSI() {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "iw" command for getting RSSI
  cmd = "iw wlan0 link | sed -n 's/^.*signal:*//p' | cut -d ' ' -f 2";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  if (!cmd_return.empty()) {
    return std::stoi(cmd_return,nullptr,0);
  } else {
    return 0;
  }
}

//  Get Encryption for the STA mode
std::string WiFiControl::Private::GetStaConnectedEncryption() {
  std::string cmd;
  std::string cmd_return, cmd_return_1;

  // We're currently using "wpa_cli" for getting the Key mgmt
  cmd = "wpa_cli status | sed -n 's/^key_mgmt=:*//p'";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  // We're currently using "wpa_cli" for getting the group cipher
  cmd = "wpa_cli status | sed -n 's/^group_cipher=:*//p'";
  cmd_return_1 = ExecuteSystemReturn(cmd.c_str());
  cmd_return_1.erase(std::remove(cmd_return_1.begin(),cmd_return_1.end(),'\n'), cmd_return_1.end());

  // Return encryption as "key_mgmt(group cipher)"
  cmd_return += "(" + cmd_return_1 + ")" ;

  return cmd_return;
}

//  Get AP mode frequency
int WiFiControl::Private::GetAPModeFrequency(const std::string &conn_interface) {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "hostapd_cli" for getting the AP mode frequency
  cmd = "hostapd_cli -i";
  cmd += " " + conn_interface + " status | sed -n 's/^freq=*//p'";
  cmd_return = ExecuteSystemReturn(cmd.c_str());

  if (!cmd_return.empty()) {
    return std::stoi(cmd_return,nullptr,0);
  } else {
    return 0;
  }
}

//  Get AP mode SSID
std::string WiFiControl::Private::GetAPModeSSID(const std::string &conn_interface) {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "hostapd_cli" for getting the AP mode SSID
  cmd = "hostapd_cli -i";
  cmd += " " + conn_interface + " status | sed -n 's/^ssid=*//p' | cut -d \"=\" -f 2";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return cmd_return;
}

//  Get AP mode BSSID
std::string WiFiControl::Private::GetAPModeBSSID(const std::string &conn_interface) {
  std::string cmd;
  std::string cmd_return;

  // We're currently using "hostapd_cli" for getting the AP mode BSSID
  cmd = "hostapd_cli -i";
  cmd += " " + conn_interface + " status | sed -n 's/^bssid=*//p' | cut -d \"=\" -f 2";
  cmd_return = ExecuteSystemReturn(cmd.c_str());
  cmd_return.erase(std::remove(cmd_return.begin(),cmd_return.end(),'\n'), cmd_return.end());

  return (boost::to_upper_copy<std::string>(cmd_return));
}

}  //  namespace wifi
}  //  namespace connectivity
}  //  namespace adk
