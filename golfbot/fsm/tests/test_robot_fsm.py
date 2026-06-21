import unittest
from fsm import FSMFactory

class FSMFactoryTestCase(unittest.TestCase):
    def test_fsm_factory_creation(self):
        self.assertIsNotNone(FSMFactory.create_robot_fsm())

if __name__ == '__main__':
    unittest.main()
