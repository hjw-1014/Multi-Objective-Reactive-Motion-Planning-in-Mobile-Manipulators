######
from icecream import ic
import xml.etree.cElementTree as ET
tree = ET.ElementTree(file='xml_test.xml')
ic(tree)

root = tree.getroot()
ic(root)
ic(root.tag, root.attrib)

for child in root:
    print(child.tag, child.attrib)

size = root[4]
ic(size)

for child in size:
    print(child.tag, child.text)
