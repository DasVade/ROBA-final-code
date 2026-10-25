"""Stage unchanged sketches and a locally supplied Vive dependency for compilation."""
from pathlib import Path
import argparse,shutil
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--vive-source',type=Path,required=True)
p.add_argument('--output',type=Path,default=ROOT/'outputs/firmware')
a=p.parse_args()
for name in ['vive510.h','vive510.cpp']:
    if not (a.vive_source/name).is_file():raise FileNotFoundError(a.vive_source/name)
for name in ['master','slave']:
    target=a.output/name;target.mkdir(parents=True,exist_ok=True)
    shutil.copy2(ROOT/name/(name+'.ino'),target/(name+'.ino'))
for name in ['vive510.h','vive510.cpp']:shutil.copy2(a.vive_source/name,a.output/'master'/name)
print('Unchanged source staged; dependency credits retained. No device upload performed.')
