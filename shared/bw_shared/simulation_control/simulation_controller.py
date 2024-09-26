import logging
import time
from threading import Event
from typing import Generator

import rospy
from bw_interfaces.msg import ConfigureSimulation, SimulationScenarioLoadedEvent
from bw_interfaces.msg import Labels as LabelMsg
from std_msgs.msg import String


class SimulationController:
    def __init__(
        self,
        scenario_loaded_event: Event,
        configuration_acknowledged_event: Event,
        configure_simulation_pub: rospy.Publisher,
        select_scenario_pub: rospy.Publisher,
    ) -> None:
        self.scenario_loaded_event = scenario_loaded_event
        self.configuration_acknowledged_event = configuration_acknowledged_event
        self.configure_simulation_pub = configure_simulation_pub
        self.select_scenario_pub = select_scenario_pub
        self.logger = logging.getLogger("perception")

    def configure_simulation(self, simulation_config: ConfigureSimulation) -> None:
        # publish scenario and objectives. Start scenario.
        self.logger.info("Starting scenario.")
        rospy.loginfo("Starting scenario.")
        self.configure_simulation_pub.publish(simulation_config)
        if not self.configuration_acknowledged_event.wait(timeout=1.0):
            raise RuntimeError("Timed out waiting for configuration acknowledgment.")
        self.configuration_acknowledged_event.clear()

        scenario_name = simulation_config.scenario.name
        self.logger.info(f"Selecting scenario: {scenario_name}")
        rospy.loginfo(f"Selecting scenario: {scenario_name}")
        for _ in range(3):
            self.select_scenario_pub.publish(scenario_name)
            if self.scenario_loaded_event.wait(timeout=1.0):
                break
        else:
            raise RuntimeError("Timed out waiting for scenario to load.")
        self.scenario_loaded_event.clear()
        self.logger.info(f"Scenario {scenario_name} loaded.")
        rospy.loginfo(f"Scenario {scenario_name} loaded.")

    def wait_for_timer(self, duration: float) -> Generator[float, None, None]:
        start_time = time.monotonic()
        self.logger.info(f"Waiting for {duration} seconds.")
        rospy.loginfo(f"Waiting for {duration} seconds.")
        while True:
            now = time.monotonic()
            dt = now - start_time
            if dt >= duration:
                break
            if rospy.is_shutdown():
                break
            yield dt
            rospy.sleep(0.1)


def event_callback(event: Event) -> None:
    event.set()


def make_simulation_controller() -> SimulationController:
    scenario_loaded_event = Event()
    configuration_acknowledged_event = Event()

    rospy.Subscriber(
        "/simulation/scenario_loaded",
        SimulationScenarioLoadedEvent,
        lambda msg, event=scenario_loaded_event: event_callback(event),
        queue_size=1,
    )
    rospy.Subscriber(
        "/simulation/acknowledge_configuration",
        LabelMsg,
        lambda msg, event=configuration_acknowledged_event: event_callback(event),
        queue_size=1,
    )
    configure_simulation_pub = rospy.Publisher("/simulation/add_configuration", ConfigureSimulation, queue_size=1)
    select_scenario_pub = rospy.Publisher("/simulation/scenario_selection", String, queue_size=1)
    time.sleep(2.0)  # wait for publishers to connect

    return SimulationController(
        scenario_loaded_event, configuration_acknowledged_event, configure_simulation_pub, select_scenario_pub
    )
