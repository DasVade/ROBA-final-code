"""Software-only driver-station behavior checks; no robot, network or joystick initialization."""
import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import Controller_Python as driver
class DriverTests(unittest.TestCase):
    def test_deadzone_and_dominant_axis(self):
        for xy,want in [((0,0),'S'),((.24,-.24),'S'),((.25,0),'R'),((0,-.25),'F'),((-.8,.2),'L'),((.8,.2),'R'),((.2,-.8),'F'),((.2,.8),'B'),((.8,.8),'B'),((-.8,-.8),'F')]:
            with self.subTest(xy=xy):self.assertEqual(driver.get_direction_from_axes(*xy),want)
    def test_udp_encoding_and_destination(self):
        class Capture:
            def sendto(self,data,address):self.packet=(data,address)
        capture=Capture();driver.send(capture,' TARGET2:1,2,3,4  \n')
        self.assertEqual(capture.packet,(b'TARGET2:1,2,3,4\n',('192.168.4.1',4210)))
    def test_original_sources(self):
        import hashlib,json
        root=Path(__file__).resolve().parents[1]
        for item in json.loads((root/'docs/source_manifest.json').read_text()):
            self.assertEqual(hashlib.sha256((root/item['path']).read_bytes()).hexdigest(),item['sha256'],item['path'])
if __name__=='__main__':unittest.main()
