"""
ros2_lingua.dispatch_config
-----------------------------
DispatchConfig lets you wire up existing robot actions/services to
ros2_lingua without subclassing DispatcherNode.

Instead of overriding _call_action(), you declare a mapping from
capability names to your robot's existing action/service types and
a parameter adapter function.

This is Level 2 integration — for robots that already have their
own typed interfaces and don't want to implement LinguaAction.

Usage:
    from ros2_lingua.dispatch_config import DispatchConfig, ActionMapping
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped

    def navigate_adapter(params: dict) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = LOCATION_MAP[params["location_name"]][0]
        goal.pose.pose.position.y = LOCATION_MAP[params["location_name"]][1]
        return goal

    config = DispatchConfig()
    config.register_action(
        capability_name="navigate_to_location",
        action_type=NavigateToPose,
        action_name="navigate_to_pose",
        goal_adapter=navigate_adapter,
        timeout_sec=30.0,
    )
"""

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional, Type


@dataclass
class ActionMapping:
    """
    Maps one capability to an existing typed ROS 2 action.

    Fields:
        capability_name:  The lingua capability this mapping handles
        action_type:      The ROS 2 action type class (e.g. NavigateToPose)
        action_name:      The action server name (e.g. "navigate_to_pose")
        goal_adapter:     Function: dict → action Goal message
        result_checker:   Optional function: action Result → bool (success?)
        timeout_sec:      How long to wait for the action to complete
    """
    capability_name: str
    action_type: Any
    action_name: str
    goal_adapter: Callable[[Dict], Any]
    result_checker: Optional[Callable[[Any], bool]] = None
    timeout_sec: float = 30.0


@dataclass
class ServiceMapping:
    """
    Maps one capability to an existing typed ROS 2 service.

    Fields:
        capability_name:  The lingua capability this mapping handles
        service_type:     The ROS 2 service type class
        service_name:     The service name
        request_adapter:  Function: dict → service Request message
        result_checker:   Optional function: service Response → bool
        timeout_sec:      How long to wait for the service response
    """
    capability_name: str
    service_type: Any
    service_name: str
    request_adapter: Callable[[Dict], Any]
    result_checker: Optional[Callable[[Any], bool]] = None
    timeout_sec: float = 10.0


class DispatchConfig:
    """
    A configuration object that maps capability names to existing
    typed ROS 2 actions and services.

    Pass an instance to DispatcherNode at construction time:
        dispatcher = DispatcherNode(dispatch_config=config)

    Or set it after construction:
        node.set_dispatch_config(config)

    Example — mapping nav2's NavigateToPose to a lingua capability:

        from nav2_msgs.action import NavigateToPose

        LOCATIONS = {
            "table": (1.5, 0.8),
            "door":  (3.2, 0.0),
        }

        def nav_adapter(params):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"
            x, y = LOCATIONS.get(params["location_name"], (0, 0))
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            return goal

        config = DispatchConfig()
        config.register_action(
            "navigate_to_location",
            NavigateToPose,
            "navigate_to_pose",
            goal_adapter=nav_adapter,
        )
    """

    def __init__(self):
        self._actions:   Dict[str, ActionMapping]  = {}
        self._services:  Dict[str, ServiceMapping] = {}

    def register_action(
        self,
        capability_name: str,
        action_type: Any,
        action_name: str,
        goal_adapter: Callable[[Dict], Any],
        result_checker: Optional[Callable[[Any], bool]] = None,
        timeout_sec: float = 30.0,
    ) -> "DispatchConfig":
        """
        Register a mapping from a capability to an existing typed action.

        Returns self for chaining:
            config.register_action(...).register_action(...).register_service(...)
        """
        self._actions[capability_name] = ActionMapping(
            capability_name=capability_name,
            action_type=action_type,
            action_name=action_name,
            goal_adapter=goal_adapter,
            result_checker=result_checker,
            timeout_sec=timeout_sec,
        )
        return self

    def register_service(
        self,
        capability_name: str,
        service_type: Any,
        service_name: str,
        request_adapter: Callable[[Dict], Any],
        result_checker: Optional[Callable[[Any], bool]] = None,
        timeout_sec: float = 10.0,
    ) -> "DispatchConfig":
        """Register a mapping from a capability to an existing typed service."""
        self._services[capability_name] = ServiceMapping(
            capability_name=capability_name,
            service_type=service_type,
            service_name=service_name,
            request_adapter=request_adapter,
            result_checker=result_checker,
            timeout_sec=timeout_sec,
        )
        return self

    def get_action(self, capability_name: str) -> Optional[ActionMapping]:
        return self._actions.get(capability_name)

    def get_service(self, capability_name: str) -> Optional[ServiceMapping]:
        return self._services.get(capability_name)

    def has(self, capability_name: str) -> bool:
        return capability_name in self._actions or capability_name in self._services

    def registered_capabilities(self) -> list:
        return list(self._actions.keys()) + list(self._services.keys())
