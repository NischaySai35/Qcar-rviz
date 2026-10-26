#!/usr/bin/python3
"""Build a fake eSync package for local testing, shaped like the vendor's
qcar-pkg-v0.1.0.x: a zip (renamed .x) holding class.json + manifest.xml.

  /usr/bin/python3 make_test_package.py classes/new.json
      -> esync-campaigns/qcar-pkg-v<version>.x

The watcher in yolo_ota2_demo.py picks it up exactly as if eSync had
downloaded it.
"""
import json
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))

MANIFEST = """<xl4_pkg_update_manifest>
    <package>qcar-pkg</package>
    <name>Qcar-pkg</name>
    <type>/SDK/C_AGENT_1</type>
    <version>v{version}</version>
</xl4_pkg_update_manifest>
"""


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    src = sys.argv[1]
    with open(src) as fh:
        version = str(json.load(fh).get('version', '0.0.0'))
    out_dir = os.path.join(HERE, 'esync-campaigns')
    os.makedirs(out_dir, exist_ok=True)
    final = os.path.join(out_dir, f'qcar-pkg-v{version}.x')
    # Write under another name, then rename, so the watcher never sees a
    # half-written file.
    tmp = final + '.part'
    with zipfile.ZipFile(tmp, 'w') as zf:
        zf.write(src, 'class.json')
        zf.writestr('manifest.xml', MANIFEST.format(version=version))
    os.replace(tmp, final)
    print(f'Wrote {final}')


if __name__ == '__main__':
    main()
