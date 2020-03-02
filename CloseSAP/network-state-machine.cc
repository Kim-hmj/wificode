/*
Copyright (c) 2019, The Linux Foundation. All rights reserved.

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
#include "network-state-machine.h"

// Boost library supports only up to 50 state transitions (rows), so increase
// it to 100 as connectivity-manager NSM requires more than 50 transitions.
// See /usr/include/boost/mpl/vector/vector50 for info on how the default
// is set to 50.
// We follow similar approach (in the below code) to extend it further.
#include <boost/mpl/vector/vector50.hpp>
#include <boost/preprocessor/iterate.hpp>
namespace boost { namespace mpl {
#   define BOOST_PP_ITERATION_PARAMS_1 \
    (3,(51, 100, <boost/mpl/vector/aux_/numbered.hpp>))
#   include BOOST_PP_ITERATE()
}}

#include <boost/function.hpp>
#include <boost/smart_ptr.hpp>
#include <sys/time.h>

#include <memory>

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/tools.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/operator.hpp>

namespace msm = boost::msm;
namespace msmf = boost::msm::front;
namespace mpl = boost::mpl;
namespace euml = msmf::euml;

namespace adk {
namespace connectivity {
namespace nsm {

/**
 * @struct BaseEvent
 * @brief Base event for the network state machine.
 */
struct BaseEvent {
  BaseEvent(NetworkStateEventID id_input): id(id_input) {};
  NetworkStateEventID id;
};

/**
 * @struct WiFiEnableEvent
 * @brief Wi-Fi enable event for the network state machine.
 */
struct WiFiEnableEvent: public BaseEvent {
  WiFiEnableEvent(): BaseEvent(kNSMWiFiEnableEventID) {};
};

/**
 * @struct WiFiDisableEvent
 * @brief Wi-Fi disable event for the network state machine.
 */
struct WiFiDisableEvent: public BaseEvent {
  WiFiDisableEvent() : BaseEvent(kNSMWiFiDisableEventID) {};
};

/**
 * @struct WiFiConnectedEvent
 * @brief Wi-Fi connected event for the network state machine.
 */
struct WiFiConnectedEvent: public BaseEvent {
  WiFiConnectedEvent() : BaseEvent(kNSMWiFiConnectedEventID) {};
};

/**
 * @struct WiFiOnboardEvent
 * @brief Wi-Fi onboard event for the network state machine.
 */
struct WiFiOnboardEvent: public BaseEvent {
  WiFiOnboardEvent() : BaseEvent(kNSMWiFiOnboardEventID) {};
};

/**
 * @struct WiFiScanEvent
 * @brief Wi-Fi scan event for the network state machine.
 */
struct WiFiScanEvent: public BaseEvent {
  WiFiScanEvent() : BaseEvent(kNSMWiFiScanEventID) {};
};

/**
 * @struct WiFiConnectEvent
 * @brief Wi-Fi connect event for the network state machine.
 */
struct WiFiConnectEvent: public BaseEvent {
  WiFiConnectEvent() : BaseEvent(kNSMWiFiConnectEventID) {};
};

/**
 * @struct WiFiConnectionTimedoutEvent
 * @brief Wi-Fi connection timeout event for the network state machine.
 */
struct WiFiConnectionTimedoutEvent: public BaseEvent {
  WiFiConnectionTimedoutEvent() : BaseEvent(kNSMWiFiConnectionTimedoutEventID) {};
};

/**
 * @struct WiFiCompleteOnboardingEvent
 * @brief Wi-Fi complete onboarding event for the network state machine.
 */
struct WiFiCompleteOnboardingEvent : public BaseEvent {
  WiFiCompleteOnboardingEvent() : BaseEvent(kNSMWiFiCompleteOnboardingEventID) {};
};

/**
 * @struct WiFiMlanLeadEnabledEvent
 * @brief Wi-Fi Mlan lead event for the network state machine.
 */
struct WiFiMlanLeadEnabledEvent: public BaseEvent {
  WiFiMlanLeadEnabledEvent() : BaseEvent(kNSMWiFiMlanLeadEnabledEventID) {};
};

/**
 * @struct WiFiMlanOnboardingStartEvent
 * @brief Wi-Fi Mlan onboarding start event for the network state machine.
 */
struct WiFiMlanOnboardingStartEvent : public BaseEvent {
  WiFiMlanOnboardingStartEvent() : BaseEvent(kNSMWiFiMlanOnboardingStartEventID) {};
};

/**
 * @struct WiFiMlanOnboardingStopEvent
 * @brief Wi-Fi Mlan onboarding stop event for the network state machine.
 */
struct WiFiMlanOnboardingStopEvent  : public BaseEvent {
  WiFiMlanOnboardingStopEvent() : BaseEvent(kNSMWiFiMlanOnboardingStopEventID) {};
};

/**
 * @struct WiFiMlanStopEvent
 * @brief Wi-Fi Mlan stop event for the network state machine.
 */
struct WiFiMlanStopEvent  : public BaseEvent {
  WiFiMlanStopEvent() : BaseEvent(kNSMWiFiMlanStopEventID) {};
};

/**
 * @struct WiFiMlanStartLeadEvent
 * @brief Wi-Fi Mlan lead start event for the network state machine.
 */
struct WiFiMlanStartLeadEvent : public BaseEvent {
  WiFiMlanStartLeadEvent() : BaseEvent(kNSMWiFiMlanStartLeadEventID) {};
};

/**
 * @struct WiFiMlanStartSlaveEvent
 * @brief Wi-Fi Mlan slave start event for the network state machine.
 */
struct WiFiMlanStartSlaveEvent : public BaseEvent {
  WiFiMlanStartSlaveEvent() : BaseEvent(kNSMWiFiMlanStartSlaveEventID) {};
};

/**
 * @struct EthernetConnectedEvent
 * @brief Ethernet connected event for the network state machine.
 */
struct EthernetConnectedEvent : public BaseEvent {
  EthernetConnectedEvent() : BaseEvent(kNSMEthernetConnectedEventID) {};
};

/**
 * @struct EthernetDisconnectedEvent
 * @brief Ethernet disconnected event for the network state machine.
 */
struct EthernetDisconnectedEvent : public BaseEvent {
  EthernetDisconnectedEvent() : BaseEvent(kNSMEthernetDisconnectedEventID) {};
};


/**
 * @struct NetworkStateMachineInternal
 * @brief Class containing the NetworkStateMachine using boost MSM.
 * Handles states for Wi-Fi onboarding, MLAN group formation,
 * switching between Wi-Fi and Ethernet, etc. in connectivity-manager.
 */
struct NetworkStateMachineInternal: public msm::front::state_machine_def<NetworkStateMachineInternal> {

  /**
   * @brief Initialise network state machine with boost MSM front-end functionality.
   * @param[in] wifi_control Pointer to Wi-Fi control object.
   */
  NetworkStateMachineInternal(std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control);

  /**
   * @brief Get the current network state machine state
   * @param[out] Current network state machine state
   */
  NetworkState GetCurrentState();

  /**
   * @brief Destructor for network state machine
   */
  ~NetworkStateMachineInternal() {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::exiting");
  };

  /**
   * Network state machine states (boost MSM)
   */

  /**
   * @Init
   * @brief Init state for network state machine.
   * It enters into this state as the default state during reset
   */
  struct Init: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::Init_on_entry()");
      fsm.current_state_ = kNSMInit;
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::Init_on_exit()");
    }
  };

  /**
   * @WiFiDisabled
   * @brief WiFiDisabled state for network state machine.
   * It enters into this state when Wi-Fi gets disabled.
   */
  struct WiFiDisabled: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiDisabled_on_entry()");
      fsm.current_state_ = kNSMWiFiDisabled;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiDisabled_on_exit()");
    }
  };

  /**
   * @WiFiOnboarding
   * @brief WiFiOnboarding state for network state machine.
   * It enters into this state when Wi-Fi (STA mode) is getting onboarded
   * to home-AP.
   */
  struct WiFiOnboarding: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiOnboarding_on_entry()");
      fsm.current_state_ = kNSMWiFiOnboarding;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiOnboarding_on_exit()");
    }
  };

  /**
   * @WiFiHomeSta
   * @brief WiFiHomeSta state for network state machine.
   * It enters into this state when Wi-Fi (STA mode) is onboarded (connected)
   * to home-AP.
   */
  struct WiFiHomeSta: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiHomeSTA_on_entry()");
      fsm.current_state_ = kNSMWiFiHomeSta;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiHomeSTA_on_exit()");
    }
  };

  /**
   * @WiFiHomeSta_WiFiMlanAp
   * @brief WiFiHomeSta_WiFiMlanAp state for network state machine.
   * It enters into this state (typically for soundbar) when Wi-Fi (STA mode)
   * is onboarded (connected)to home-AP + device has started
   * a MLAN AP for the satellite speakers.
   */
  struct WiFiHomeSta_WiFiMlanAp: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiHomeSta_WiFiMlanAp_on_entry()");
      fsm.current_state_ = kNSMWiFiHomeSta_WiFiMlanAp;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiHomeSta_WiFiMlanAp_on_exit()");
    }
  };

  /**
   * @WiFiMlanOnboarding
   * @brief WiFiMlanOnboarding state for network state machine.
   * It enters into this state for onboarding to a MLAN (soundbar) network.
   */
  struct WiFiMlanOnboarding: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanOnboardingStart_on_entry()");
      fsm.current_state_ = kNSMWiFiMlanOnboarding;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanOnboardingStart_on_exit()");
    }
  };

  /**
   * @WiFiMlanSta
   * @brief WiFiMlanSta state for network state machine.
   * It enters into this state (typically satellite speaker) after onboarded/
   * connected to MLAN AP (soundbar network).
   *
   * NOTE: we're not supporting satellite speaker to be connected over Ethernet
   * to soundbar for now. Hence, we're not checking for Ethernet connectivity
   * in this state.
   *
   */
  struct WiFiMlanSta: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanSta_on_entry()");
      fsm.current_state_ = kNSMWiFiMlanSta;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanSta_on_exit()");
    }
  };

  /**
   * @EthernetHome
   * @brief EthernetHome state for network state machine.
   * It enters into this state (typically soundbar speaker) when Ethernet
   * is connected to home router (AP).
   */
  struct EthernetHome: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::EthernetHome_on_entry()");
      fsm.current_state_ = kNSMEthernetHome;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::EthernetHome_on_exit()");
    }
  };

  /**
   * @EthernetHome_WiFiMlanAp
   * @brief EthernetHome_WiFiMlanAp state for network state machine.
   * It enters into this state (typically soundbar speaker) when Ethernet
   * is connected to home network + MLAN AP
   *
   */
  struct EthernetHome_WiFiMlanAp: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::EthernetHome_WiFiMlanAp_on_entry()");
      fsm.current_state_ = kNSMEthernetHome_WiFiMlanAp;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::EthernetHome_WiFiMlanAp_on_exit()");
    }
  };

  /**
   * @WiFiMlanAp
   * @brief WiFiMlanAp state for network state machine.
   * It enters into this state (typically soundbar speaker) when the device is
   * enabled in MLAN AP mode, but without onboarded to home-AP over Wi-Fi
   * and Ethernet not connected.
   *
   */
  struct WiFiMlanAp: public msm::front::state<> {

    template <class Event, class Fsm>
    void on_entry(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanAp_on_entry()");
      fsm.current_state_ = kNSMWiFiMlanAp;
      fsm.wifi_control_->SendNetworkInfo();
    }

    template <class Event, class Fsm>
    void on_exit(Event const&, Fsm &fsm) {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::WiFiMlanAp_on_exit()");
    }
  };

  // Initialisation state (after a device reset)
  typedef Init initial_state;

  // Boost MSM actions

  /**
   * @actionWiFiDisable
   * @brief Wi-Fi disable action during the state transition.
   */
  struct actionWiFiDisable {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiDisable()");
      fsm.wifi_control_->Disable();
    }
  };

  /**
   * @actionWiFiStop
   * @brief Stop Wi-Fi activities during the state transition.
   */
  struct actionWiFiStop {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiStop()");
      fsm.wifi_control_->Stop();
    }
  };

  /**
   * @actionWiFiEnableInOnboardingMode
   * @brief Re-enable Wi-Fi in onboarding mode
   */
  struct actionWiFiEnableInOnboardingMode {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiEnableInOnboardingMode()");
      fsm.wifi_control_->EnableInOnboardMode();
    }
  };


  /**
   * @actionSetWiFiEnableInConfig
   * @brief Set Wi-Fi enable configuration
   */
  struct actionSetWiFiEnableInConfig {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionSetWiFiEnableInConfig()");
      fsm.wifi_control_->SetEnableInConfig();
    }
  };

  /**
   * @actionWiFiConnect
   * @brief Wi-Fi connect action during the state transition.
   */
  struct actionWiFiConnect {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiConnect()");
      fsm.wifi_control_->Connect();
    }
  };

  /**
   * @actionWiFiScan
   * @brief Wi-Fi scan action during the state transition.
   */
  struct actionWiFiScan {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiScan()");
      fsm.wifi_control_->Scan();
    }
  };

  /**
   * @actionWiFiEnableInHomeStaMode
   * @brief Re-enable Wi-Fi in home-STA mode (connected to home-AP)
   */
  struct actionWiFiEnableInHomeStaMode{
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiEnableInHomeStaMode()");
      fsm.wifi_control_->EnableInHomeStaMode();
    }
  };

  /**
   * @actionWiFiEnableInHomeStaMlanApMode
   * @brief Re-enable Wi-Fi in STA (connected to home-AP) + MLAN AP mode
   */
  struct actionWiFiEnableInHomeStaMlanApMode{
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiEnableInHomeStaMlanApMode()");
      fsm.wifi_control_->EnableInHomeStaMlanApMode();
    }
  };

  /**
   * @actionWiFiEnableInMlanApMode
   * @brief Re-enable Wi-Fi in MLAN AP mode
   */
  struct actionWiFiEnableInMlanApMode{
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiEnableInMlanApMode()");
      fsm.wifi_control_->EnableInMlanApMode();
    }
  };

  /**
   * @actionWiFiOnboarded
   * @brief Wi-Fi onboarded (connected to home-AP) action during
   * the state transition.
   */
  struct actionWiFiOnboarded {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiOnboarded()");
      fsm.wifi_control_->SetOnboardedMode();
    }
  };

  /**
   * @actionWiFiReconnectToHomeAp
   * @brief Wi-Fi re-connect with home-AP (without changing the current STA/AP mode)
   */
  struct actionWiFiReconnectToHomeAp {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiReconnectToHomeAp()");
      fsm.wifi_control_->StaReconnectToHomeAP();
    }
  };

  /**
   * @actionWiFiStartOnboardCompTimer
   * @brief Wi-Fi start onboarding complete timer during the state transition.
   */
  struct actionWiFiStartOnboardCompTimer {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiStartOnboardCompTimer()");
      fsm.wifi_control_->StartWaitingForOnboardCompleteMessage();
    }
  };

  /**
   * @actionWiFiEnableInMlanSlaveMode
   * @brief Re-enable Wi-Fi in in MLAN slave (STA) mode
   */
  struct actionWiFiEnableInMlanSlaveMode {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiEnableInMlanSlaveMode()");
      fsm.wifi_control_->EnableInMlanStaMode();
    }
  };

  /**
   * @actionWiFiMlanOnboardingConnected
   * @brief Wi-Fi Mlan Onboarding connected action during the state transition.
   */
  struct actionWiFiMlanOnboardingConnected {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanOnboardingConnected()");
      fsm.wifi_control_->SendMlanOnboardingStatus();
    }
  };

  /**
   * @actionWiFiMlanOnboardingStop
   * @brief Wi-Fi Mlan Onboarding stop action during the state transition.
   */
  struct actionWiFiMlanOnboardingStop {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanOnboardingStop()");
      fsm.wifi_control_->StopMlanOnboarding();
    }
  };

  /**
   * @actionWiFiHomeStaMlanLeadStart
   * @brief Wi-Fi STA (home-AP) + Mlan start lead action during the state transition.
   */
  struct actionWiFiHomeStaMlanLeadStart {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiHomeStaMlanLeadStart()");
      fsm.wifi_control_->StartHomeStaMlanLead();
    }
  };

  /**
   * @actionWiFiMlanLeadStart
   * @brief Wi-Fi Mlan start lead action during the state transition.
   */
  struct actionWiFiMlanLeadStart {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanLeadStart()");
      fsm.wifi_control_->StartMlanLead();
    }
  };

  /**
   * @actionWiFiMlanSlaveStart
   * @brief Wi-Fi Mlan slave lead action during the state transition.
   */
  struct actionWiFiMlanSlaveStart {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanSlaveStart()");
      fsm.wifi_control_->StartMlanSlave();
    }
  };

  /**
   * @actionWiFiMlanSlaveConnected
   * @brief Wi-Fi Mlan slave connected action during the state transition.
   */
  struct actionWiFiMlanSlaveConnected {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanSlaveConnected()");
      fsm.wifi_control_->SetMlanSlaveConnected();
    }
  };

  /**
   * @actionWiFiResetMlanState
   * @brief Wi-Fi reset Mlan slave state
   */
  struct actionWiFiResetMlanState {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiResetMlanState()");
      fsm.wifi_control_->ResetMlanState();
    }
  };

  /**
   * @actionWiFiMlanStop
   * @brief Wi-Fi Mlan stop action during the state transition.
   */
  struct actionWiFiMlanStop {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiMlanStop()");
      fsm.wifi_control_->StopMlan();
    }
  };

  /**
   * @actionWiFiStartMlanOnboarding
   * @brief Wi-Fi start MLAN onboarding during the state transition.
   */
  struct actionWiFiStartMlanOnboarding {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionWiFiStartMlanOnboarding()");
      fsm.wifi_control_->StartMlanOnboarding();
    }
  };

  /**
   * @actionSendWiFiEnabled
   * @brief Send Wi-Fi enabled status
   */
  struct actionSendWiFiEnabled {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionSendWiFiEnabled()");
      fsm.wifi_control_->SendEnabledStatus(true);
    }
  };

  /**
   * @actionSendWiFiDisabled
   * @brief Send Wi-Fi disabled status
   */
  struct actionSendWiFiDisabled {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      void operator()(const Evt &evt, Fsm &fsm, SourceState &source, TargetState &target) const {
      ADK_LOG_DEBUG("NetworkStateMachineInternal::actionSendWiFiDisabled()");
      fsm.wifi_control_->SendDisabledStatus(true);
    }
  };

  // Boost MSM Guards

  /**
   * @IsWifiEnabled
   * @brief Guard on Wi-Fi enabled state.
   */
  struct IsWiFiEnabled {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsEnabled();
      }
  };

  /**
   * @IsWiFiOnboarded
   * @brief Guard on Wi-Fi onboarded state.
   */
  struct IsWiFiOnboarded {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsOnboarded();
      }
  };

  /**
   * @IsWiFiMlanSlave
   * @brief Guard on Wi-Fi MLAN slave state.
   */
  struct IsWiFiMlanSlave {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsMlanSlaveEnabled();
      }
  };

  /**
   * @IsWiFiMlanLead
   * @brief Guard on Wi-Fi MLAN lead state.
   */
  struct IsWiFiMlanLead {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsMlanLeadEnabled();
      }
  };

  /**
   * @IsWiFiStaConnected
   * @brief Guard on Wi-Fi STA mode connected.
   */
  struct IsWiFiStaConnected {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsWiFiStaConnected();
      }
  };

  /**
   * @IsWiFiMlanEnabled
   * @brief Guard on Wi-Fi MLAN enabled (lead/slave enabled).
   */
  struct IsWiFiMlanEnabled {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return (fsm.wifi_control_->IsMlanLeadEnabled() || fsm.wifi_control_->IsMlanSlaveEnabled());
      }
  };

  /**
   * @IsEthernetUp
   * @brief Guard on Ethernet up (connected in the device)
   */
  struct IsEthernetUp {
    template <class Fsm,class Evt,class SourceState,class TargetState>
      bool operator()(const Evt&, Fsm& fsm, SourceState&, TargetState&) const {
        return fsm.wifi_control_->IsEthernetUp();
      }
  };

  /**
   * @transition_table
   * @brief Boost MSM state transition table.
   */

  // *** NOTE: IT'S REQUIRED TO CHANGE THE VECTOR COUNT WHEN ADDING/REMOVING TRANSITION ROWS ***
  struct transition_table : mpl::vector83<

  // msmf::Row<Start, Event, Next,
  //           Action,
  //           Guard>

    // INIT state
     msmf::Row<Init, WiFiEnableEvent, WiFiMlanSta,
               actionWiFiEnableInMlanSlaveMode,
               IsWiFiMlanSlave >,

     msmf::Row<Init, WiFiEnableEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, euml::Not_<IsWiFiOnboarded>>, euml::Not_<IsWiFiMlanEnabled>> >,

     msmf::Row<Init, WiFiEnableEvent, WiFiHomeSta,
               actionWiFiEnableInHomeStaMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, IsWiFiOnboarded>, euml::Not_<IsWiFiMlanEnabled>> >,

     msmf::Row<Init, WiFiEnableEvent, WiFiHomeSta_WiFiMlanAp,
               actionWiFiEnableInHomeStaMlanApMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, IsWiFiOnboarded>, IsWiFiMlanLead> >,

     msmf::Row<Init, WiFiEnableEvent, EthernetHome,
               actionWiFiStop,
               euml::And_<IsEthernetUp, euml::Not_<IsWiFiMlanEnabled>> >,

     msmf::Row<Init, WiFiEnableEvent, EthernetHome_WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               euml::And_<IsEthernetUp, IsWiFiMlanLead> >,

     msmf::Row<Init, adk::connectivity::nsm::WiFiEnableEvent, WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<euml::Not_<IsWiFiOnboarded>, IsWiFiMlanLead> > >,

    // This transition is to ensure that Wi-Fi gets disabled during boot
    // (if it was disabled before) before changing the state
     msmf::Row<Init, WiFiDisableEvent, EthernetHome,
               actionWiFiDisable,
               IsEthernetUp >,

    // This transition is to ensure that Wi-Fi gets disabled during boot if it
    // was disabled before
     msmf::Row<Init, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               euml::Not_<IsEthernetUp> >,

    // WIFI_DISABLED state
     msmf::Row<WiFiDisabled, WiFiEnableEvent, WiFiMlanSta,
               actionWiFiEnableInMlanSlaveMode,
               IsWiFiMlanSlave >,

     msmf::Row<WiFiDisabled, WiFiEnableEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, euml::Not_<IsWiFiOnboarded>>, euml::Not_<IsWiFiMlanEnabled>> >,

     msmf::Row<WiFiDisabled, WiFiEnableEvent, WiFiHomeSta,
               actionWiFiEnableInHomeStaMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, IsWiFiOnboarded>, euml::Not_<IsWiFiMlanEnabled>> >,

     msmf::Row<WiFiDisabled, WiFiEnableEvent, WiFiHomeSta_WiFiMlanAp,
               actionWiFiEnableInHomeStaMlanApMode,
               euml::And_<euml::And_<euml::Not_<IsEthernetUp>, IsWiFiOnboarded>, IsWiFiMlanLead> >,

     msmf::Row<WiFiDisabled, WiFiEnableEvent, WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<euml::Not_<IsWiFiOnboarded>, IsWiFiMlanLead> > >,

     msmf::Row<WiFiDisabled, EthernetConnectedEvent, EthernetHome,
               msmf::none,
               msmf::none >,

    // Send the current Wi-Fi disabled state if a new disable event is received
     msmf::Row<WiFiDisabled, WiFiDisableEvent, WiFiDisabled,
               actionSendWiFiDisabled,
               msmf::none >,

     // WIFI_ONBOARDING state
     msmf::Row<WiFiOnboarding, WiFiScanEvent, msmf::none,
               actionWiFiScan,
               msmf::none >,

     msmf::Row<WiFiOnboarding, WiFiConnectEvent, msmf::none,
               actionWiFiConnect,
               msmf::none >,

     msmf::Row<WiFiOnboarding, WiFiConnectedEvent, msmf::none,
              actionWiFiStartOnboardCompTimer,
               msmf::none >,

     msmf::Row<WiFiOnboarding, WiFiCompleteOnboardingEvent, WiFiHomeSta,
               msmf::ActionSequence_<mpl::vector<actionWiFiOnboarded>>,
              euml::And_<euml::Not_<IsWiFiMlanLead>,IsWiFiStaConnected> >,

     msmf::Row<WiFiOnboarding, WiFiCompleteOnboardingEvent, WiFiHomeSta_WiFiMlanAp,
               msmf::ActionSequence_<mpl::vector<actionWiFiOnboarded, actionWiFiEnableInHomeStaMlanApMode>>,
               euml::And_<IsWiFiMlanLead,IsWiFiStaConnected>>,

     msmf::Row<WiFiOnboarding, WiFiMlanStartSlaveEvent, WiFiMlanSta,
               actionWiFiMlanSlaveStart,
               msmf::none >,

     msmf::Row<WiFiOnboarding, EthernetConnectedEvent, EthernetHome,
               actionWiFiStop,
               euml::Not_<IsWiFiMlanEnabled> >,

     msmf::Row<WiFiOnboarding, EthernetConnectedEvent, EthernetHome_WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               IsWiFiMlanLead >,

     msmf::Row<WiFiOnboarding, WiFiDisableEvent, EthernetHome,
               actionWiFiDisable,
               IsEthernetUp >,

     msmf::Row<WiFiOnboarding, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               euml::Not_<IsEthernetUp> >,

    msmf::Row<WiFiOnboarding, WiFiOnboardEvent, msmf::none,
              actionWiFiEnableInOnboardingMode,
              msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiOnboarding, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // WIFI_HOME_STA state
     msmf::Row<WiFiHomeSta, WiFiMlanOnboardingStartEvent, WiFiMlanOnboarding,
               actionWiFiStartMlanOnboarding,
               msmf::none >,

     msmf::Row<WiFiHomeSta, WiFiMlanStartLeadEvent, WiFiHomeSta_WiFiMlanAp,
               actionWiFiHomeStaMlanLeadStart,
               msmf::none >,

     msmf::Row<WiFiHomeSta, WiFiMlanStartSlaveEvent, WiFiMlanSta,
               actionWiFiMlanSlaveStart,
               msmf::none >,

     msmf::Row<WiFiHomeSta, WiFiOnboardEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               msmf::none >,

     msmf::Row<WiFiHomeSta, EthernetConnectedEvent, EthernetHome,
               actionWiFiStop,
               msmf::none >,

     msmf::Row<WiFiHomeSta, WiFiDisableEvent, EthernetHome,
               actionWiFiDisable,
               IsEthernetUp >,

     msmf::Row<WiFiHomeSta, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               euml::Not_<IsEthernetUp> >,

    msmf::Row<WiFiHomeSta, WiFiScanEvent, msmf::none,
              actionWiFiScan,
              msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiHomeSta, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // WIFI_HOME_STA_MLAN_AP state
     msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiMlanOnboardingStartEvent, WiFiMlanOnboarding,
               actionWiFiStartMlanOnboarding,
               msmf::none >,

     msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiMlanStopEvent, WiFiHomeSta,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiEnableInHomeStaMode>>,
               msmf::none >,

     msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiOnboardEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               msmf::none >,

     msmf::Row<WiFiHomeSta_WiFiMlanAp, EthernetConnectedEvent, EthernetHome_WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               msmf::none >,

     msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               msmf::none >,

    msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiScanEvent, msmf::none,
              actionWiFiScan,
              msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiHomeSta_WiFiMlanAp, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // WIFI_MLAN_ONBOARDING state
     msmf::Row<WiFiMlanOnboarding, WiFiConnectedEvent, msmf::none,
               actionWiFiMlanOnboardingConnected,
               msmf::none >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, WiFiOnboarding,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiEnableInOnboardingMode>>,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<euml::Not_<IsWiFiMlanLead>, euml::Not_<IsWiFiOnboarded>>> >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, WiFiHomeSta,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiReconnectToHomeAp>>,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<euml::Not_<IsWiFiMlanLead>, IsWiFiOnboarded>> >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, WiFiHomeSta_WiFiMlanAp,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiReconnectToHomeAp>>,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<IsWiFiMlanLead, IsWiFiOnboarded>> >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, WiFiMlanAp,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiEnableInMlanApMode>>,
               euml::And_<euml::Not_<IsEthernetUp>, euml::And_<IsWiFiMlanLead, euml::Not_<IsWiFiOnboarded>>> >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, EthernetHome,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiStop>>,
               euml::And_<IsEthernetUp, euml::Not_<IsWiFiMlanLead>> >,

     msmf::Row<WiFiMlanOnboarding, WiFiMlanOnboardingStopEvent, EthernetHome_WiFiMlanAp,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanOnboardingStop, actionWiFiEnableInMlanApMode>>,
               euml::And_<IsEthernetUp, IsWiFiMlanLead> >,

     msmf::Row<WiFiMlanOnboarding, WiFiScanEvent, msmf::none,
               actionWiFiScan,
               msmf::none >,

     msmf::Row<WiFiMlanOnboarding, WiFiDisableEvent, EthernetHome,
               actionWiFiDisable,
               IsEthernetUp >,

     msmf::Row<WiFiMlanOnboarding, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               euml::Not_<IsEthernetUp> >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiMlanOnboarding, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // WIFI_MLAN_STA state
     msmf::Row<WiFiMlanSta, WiFiConnectedEvent, msmf::none,
               actionWiFiMlanSlaveConnected,
               euml::Not_<IsWiFiMlanSlave> >,

     msmf::Row<WiFiMlanSta, WiFiMlanStopEvent, WiFiHomeSta,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiReconnectToHomeAp>>,
               euml::And_<IsWiFiOnboarded, euml::Not_<IsEthernetUp>> >,

     msmf::Row<WiFiMlanSta, WiFiMlanStopEvent, WiFiOnboarding,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiEnableInOnboardingMode>>,
               euml::And_<euml::Not_<IsWiFiOnboarded>, euml::Not_<IsEthernetUp>> >,

     msmf::Row<WiFiMlanSta, WiFiMlanStopEvent, EthernetHome,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiStop>>,
               IsEthernetUp >,

     msmf::Row<WiFiMlanSta, WiFiScanEvent, msmf::none,
               actionWiFiScan,
               msmf::none >,

     msmf::Row<WiFiMlanSta, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiMlanSta, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // ETHERNET_LAN state
     msmf::Row<EthernetHome, EthernetDisconnectedEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               euml::And_<IsWiFiEnabled, euml::Not_<IsWiFiOnboarded>> >,

     msmf::Row<EthernetHome, EthernetDisconnectedEvent, WiFiHomeSta,
               actionWiFiEnableInHomeStaMode,
               euml::And_<IsWiFiEnabled, IsWiFiOnboarded> >,

     msmf::Row<EthernetHome, EthernetDisconnectedEvent, WiFiDisabled,
               msmf::none,
               euml::Not_<IsWiFiEnabled> >,

     msmf::Row<EthernetHome, WiFiMlanStartLeadEvent, EthernetHome_WiFiMlanAp,
               actionWiFiMlanLeadStart,
               msmf::none >,

     msmf::Row<EthernetHome, WiFiMlanStartSlaveEvent, WiFiMlanSta,
               actionWiFiMlanSlaveStart,
               msmf::none >,

     msmf::Row<EthernetHome, WiFiDisableEvent, msmf::none,
               actionWiFiDisable,
               msmf::none >,

     msmf::Row<EthernetHome, WiFiMlanOnboardingStartEvent, WiFiMlanOnboarding,
               msmf::ActionSequence_<mpl::vector<actionWiFiEnableInHomeStaMlanApMode, actionWiFiStartMlanOnboarding>>,
               msmf::none >,

     msmf::Row<EthernetHome, WiFiEnableEvent, EthernetHome_WiFiMlanAp,
               actionWiFiEnableInMlanApMode,
               IsWiFiMlanLead >,

    // Only set the Wi-Fi enable bit in the configuration (without actually enabling Wi-Fi) so that
    // Wi-Fi will be enabled in the platform when Etherent is unplugged/MLAN AP is started
     msmf::Row<EthernetHome, WiFiEnableEvent, msmf::none,
               actionSetWiFiEnableInConfig,
               euml::Not_<IsWiFiMlanLead> >,

     // ETHERNET_LAN_WIFI_MLAN_AP state
     msmf::Row<EthernetHome_WiFiMlanAp, EthernetDisconnectedEvent, WiFiHomeSta_WiFiMlanAp,
               actionWiFiEnableInHomeStaMlanApMode,
               IsWiFiOnboarded >,

     msmf::Row<EthernetHome_WiFiMlanAp, EthernetDisconnectedEvent, WiFiMlanAp,
               msmf::none,
               euml::Not_<IsWiFiOnboarded> >,

     msmf::Row<EthernetHome_WiFiMlanAp, WiFiDisableEvent, EthernetHome,
               actionWiFiDisable,
               msmf::none >,

     msmf::Row<EthernetHome_WiFiMlanAp, WiFiMlanOnboardingStartEvent, WiFiMlanOnboarding,
               msmf::ActionSequence_<mpl::vector<actionWiFiEnableInHomeStaMlanApMode, actionWiFiStartMlanOnboarding>>,
               msmf::none >,

     msmf::Row<EthernetHome_WiFiMlanAp, WiFiMlanStopEvent, EthernetHome,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiStop>>,
               msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<EthernetHome_WiFiMlanAp, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >,

     // WIFI_MLAN_AP state
     msmf::Row<WiFiMlanAp, WiFiOnboardEvent, WiFiOnboarding,
               actionWiFiEnableInOnboardingMode,
               msmf::none >,

     msmf::Row<WiFiMlanAp, EthernetConnectedEvent, EthernetHome_WiFiMlanAp,
               msmf::none,
               msmf::none >,

     msmf::Row<WiFiMlanAp, WiFiDisableEvent, WiFiDisabled,
               actionWiFiDisable,
               msmf::none >,

     msmf::Row<WiFiMlanAp, WiFiMlanOnboardingStartEvent, WiFiMlanOnboarding,
               msmf::ActionSequence_<mpl::vector<actionWiFiEnableInHomeStaMlanApMode, actionWiFiStartMlanOnboarding>>,
               msmf::none >,

     msmf::Row<WiFiMlanAp, WiFiMlanStopEvent, WiFiDisabled,
               msmf::ActionSequence_<mpl::vector<actionWiFiMlanStop, actionWiFiDisable>>,
               msmf::none >,

    // Send the current Wi-Fi enabled state if a new enable event is received
     msmf::Row<WiFiMlanAp, WiFiEnableEvent, msmf::none,
               actionSendWiFiEnabled,
               msmf::none >

  // There are a few "global" internal transitions handled via
  // no_transition to avoid copying the same Row over and over.
  > {};

  /**
   * @no_transition
   * @brief action on no state transition.
   */
  template <class FSM,class Event>
   void no_transition(Event const& event, FSM& fsm, int state) {
     ADK_LOG_DEBUG("NetworkStateMachineInternal::no_transition() from the current state for event id:%d", event.id);
     fsm.wifi_control_->SendErrorMessage(event.id);
  }

  // Wi-Fi control object pointer
  std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control_;

  // Store the current state
  NetworkState current_state_;
};

NetworkStateMachineInternal::NetworkStateMachineInternal(std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control) :
  wifi_control_(wifi_control){
  ADK_LOG_DEBUG("NetworkStateMachineInternal::NetworkStateMachineInternal()");
}

/**
 * @struct NetworkStateMachineFront
 * @brief Class containing the NetworkStateMachine front end for Boost MSM
 */
struct NetworkStateMachineFront: public msm::back::state_machine<adk::connectivity::nsm::NetworkStateMachineInternal> {
  NetworkStateMachineFront(std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control);
};

NetworkStateMachineFront::NetworkStateMachineFront(std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control) :
  msm::back::state_machine<adk::connectivity::nsm::NetworkStateMachineInternal>(wifi_control) {}

NetworkStateMachine::NetworkStateMachine(std::shared_ptr<adk::connectivity::wifi::WiFiControl> wifi_control) {
  ADK_LOG_DEBUG("NetworkStateMachine::NetworkStateMachine()");
  nsm_front_ = std::unique_ptr<adk::connectivity::nsm::NetworkStateMachineFront>(new adk::connectivity::nsm::NetworkStateMachineFront(wifi_control));
}

NetworkStateMachine::~NetworkStateMachine() {
  ADK_LOG_DEBUG("NetworkStateMachine::~NetworkStateMachine()");
}

// Get the current NSM state
NetworkState NetworkStateMachineInternal::GetCurrentState() {
  return current_state_;
}

// Process the input NSM event
void NetworkStateMachine::ProcessEvent(NetworkStateEventID nsm_event) {

  ADK_LOG_DEBUG("~NetworkStateMachine::process_event() event:%d", nsm_event);

  switch (nsm_event) {
    case kNSMWiFiEnableEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiEnableEvent());
      break;
    }
    case kNSMWiFiDisableEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiDisableEvent());
      break;
    }
    case kNSMWiFiConnectedEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiConnectedEvent());
      break;
    }
    case kNSMWiFiScanEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiScanEvent());
      break;
    }
    case kNSMWiFiConnectEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiConnectEvent());
      break;
    }
    case kNSMWiFiOnboardEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiOnboardEvent());
      break;
    }
    case kNSMWiFiConnectionTimedoutEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiConnectionTimedoutEvent());
      break;
    }
    case kNSMWiFiCompleteOnboardingEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiCompleteOnboardingEvent());
      break;
    }
    case kNSMWiFiMlanLeadEnabledEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanLeadEnabledEvent());
      break;
    }
    case kNSMWiFiMlanStartSlaveEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanStartSlaveEvent());
      break;
    }
    case kNSMWiFiMlanOnboardingStartEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanOnboardingStartEvent());
      break;
    }
    case kNSMWiFiMlanOnboardingStopEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanOnboardingStopEvent());
      break;
    }
    case kNSMWiFiMlanStopEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanStopEvent());
      break;
    }
    case kNSMWiFiMlanStartLeadEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::WiFiMlanStartLeadEvent());
      break;
    }
    case kNSMEthernetConnectedEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::EthernetConnectedEvent());
      break;
    }
    case kNSMEthernetDisconnectedEventID: {
      nsm_front_->process_event(adk::connectivity::nsm::EthernetDisconnectedEvent());
      break;
    }
    default: {
      ADK_LOG_ERROR("NetworkStateMachinel::process_event() unhandled event:%d", nsm_event);
      break;
    }
  }
}

NetworkState NetworkStateMachine::GetCurrentState() {
  return nsm_front_->current_state_;
}

}  //  namespace nsm
}  //  namespace connectivity
}  //  namespace adk

