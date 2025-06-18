
import xml.etree.ElementTree as ET

def load_detectors(add_xml_path):
    """
    Parse a SUMO additional‐files XML and return all laneAreaDetector IDs.
    
    Args:
      add_xml_path (str): path to the <additional-files> .add.xml file
    Returns:
      List[str]: list of detector IDs to poll each timestep
    """
    tree = ET.parse(add_xml_path)
    root = tree.getroot()
    return [elem.attrib["id"] for elem in root.findall("laneAreaDetector")]

