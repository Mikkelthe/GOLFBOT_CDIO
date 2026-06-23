import unittest
from utils.settings import court_settings
from utils import Conversion

# TODO: Write tests for the conversion class
class ConversionTestCase(unittest.TestCase):
    def test_px_to_world_cm(self):
        converter = Conversion()
        court_max = (court_settings.court_width, court_settings.court_height)
        image_max_with_padding_px_x = court_settings.image_width-court_settings.padding
        image_max_with_padding_px_y = court_settings.image_height-court_settings.padding
        image_max_with_padding_cm = converter.px_to_world_cm(image_max_with_padding_px_x, image_max_with_padding_px_y)

        self.assertEqual(court_max, image_max_with_padding_cm)


if __name__ == '__main__':
    unittest.main()
