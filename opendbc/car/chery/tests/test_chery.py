from opendbc.car.chery.interface import CarInterface
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData


class TestCheryInterface:
    def setup_method(self):
        # Initialize with dummy fingerprints and firmware versions
        self.fingerprints = gen_empty_fingerprint()
        self.car_fw = []  # Add any specific firmware versions if needed for your Chery

        # Get car parameters and interface
        self.car_params = CarInterface.get_params(
            "CHERY_OMODA_E5",
            self.fingerprints,
            self.car_fw,
            alpha_long=False,
            is_release=False,
            docs=False,
        )
        self.car_params_sp = CarInterface.get_params_sp(
            self.car_params,
            "CHERY_OMODA_E5",
            self.fingerprints,
            self.car_fw,
            alpha_long=False,
            docs=False,
        )
        self.car_interface = CarInterface(self.car_params, self.car_params_sp)

    def test_initialization(self):
        # Basic test to ensure the interface initializes
        assert self.car_params is not None
        assert self.car_interface is not None

    def test_update_with_empty_can(self):
        # Test updating with no CAN messages
        self.car_interface.update([])
        # You would add assertions here based on expected CarState after no messages
        # For example: assert self.car_interface.CS.vEgo == 0.0

    def test_update_with_mock_can_messages(self):
        # Example: Simulate a CAN message for vehicle speed
        # You'll need to know the message address and data format from your Chery DBC
        # For instance, if message 0x100 contains speed at bytes 0-1
        mock_can_msg = CanData(
            0x100, b"\x00\x64", 0
        )  # Example: speed 100 (adjust based on your DBC)
        self.car_interface.update([(0, [mock_can_msg])])  # (bus, [messages])
        # Assertions here to check if CarState reflects the mocked speed
        # For example: assert self.car_interface.CS.vEgo > 0.0

    def test_apply_commands(self):
        # Test applying control commands and checking generated CAN messages
        CC = structs.CarControl()
        CC.enabled = True
        CC.longActive = True
        CC.actuators.accel = 0.5  # Example acceleration
        CC = CC.as_reader()

        CC_SP = structs.CarControlSP()  # SunnyPilot specific control

        now_nanos = 0
        can_send = self.car_interface.apply(CC, CC_SP, now_nanos)
        assert len(can_send) > 0  # Expect some CAN messages to be generated
        # Further assertions to check the content of `can_send` based on your Chery's expected messages
