import unittest
from utils.settings import court_settings
from utils import Conversion

class ConversionTestCase(unittest.TestCase):
    def test_px_to_world_cm(self):
        converter = Conversion()
        court_max = (court_settings.court_width, court_settings.court_height)
        image_max_with_padding_px_x = court_settings.image_width-court_settings.padding
        image_max_with_padding_px_y = court_settings.image_height-court_settings.padding
        image_max_with_padding_cm = converter.px_to_world_cm(image_max_with_padding_px_x, image_max_with_padding_px_y)

        self.assertEqual(court_max, image_max_with_padding_cm)

    def test_world_cm_to_px(self):
        converter = Conversion()
        court_max_w = court_settings.court_width
        court_max_h = court_settings.court_height
        image_max_with_padding_px_x = court_settings.image_width - court_settings.padding
        image_max_with_padding_px_y = court_settings.image_height - court_settings.padding
        court_max_px = image_max_with_padding_px_x, image_max_with_padding_px_y
        converter_court_max_px = converter.world_cm_to_px(court_max_w, court_max_h)

        self.assertEqual(converter_court_max_px, court_max_px)


if __name__ == '__main__':
    unittest.main()
