"""MIT; Copyright (c) 2026 kiki348. Default is offline verification only."""
import argparse, hashlib, json, pathlib, re, struct, subprocess, sys, tempfile

def root():
    return pathlib.Path(sys.executable if getattr(sys,'frozen',False) else __file__).resolve().parent

def verify_bundle(folder):
    manifest=json.loads((folder/'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('chip')!='esp32s3' or manifest.get('flash_size')!='16MB':
        raise ValueError('Wrong chip or flash size')
    expected={'bootloader.bin':0,'partitions.bin':0x8000,'boot_app0.bin':0xe000,'firmware.bin':0x10000}
    files=manifest['files']
    if len(files)!=4 or {x['file'] for x in files}!=set(expected):
        raise ValueError('Need the complete four-file bundle; an OTA image is not sufficient')
    regions=[]
    for entry in files:
        name=entry['file']; address=int(entry['address'],0)
        if address!=expected[name]:raise ValueError('Unexpected address: '+name)
        raw=(folder/name).read_bytes()
        if len(raw)!=entry['size'] or hashlib.sha256(raw).hexdigest()!=entry['sha256']:
            raise ValueError('Checksum/size mismatch: '+name)
        if name in ('bootloader.bin','firmware.bin'):
            if raw[0]!=0xe9 or struct.unpack_from('<H',raw,12)[0]!=9:
                raise ValueError('Not an ESP32-S3 image: '+name)
        regions.append((address,address+len(raw)))
    regions.sort()
    if any(b>c for (a,b),(c,d) in zip(regions,regions[1:])):
        raise ValueError('Overlapping image regions')
    if regions[-1][1]>0x650000:raise ValueError('Application exceeds app0')
    partitions=(folder/'partitions.bin').read_bytes()
    parsed={}
    for pos in range(0,len(partitions),32):
        magic,typ,sub,start,size,label,flags=struct.unpack('<HBBII16sI',partitions[pos:pos+32])
        if magic==0x50aa: parsed[label.rstrip(b'\0').decode()] = (start,size)
    if parsed.get('app0')!=(0x10000,0x640000) or parsed.get('app1')!=(0x650000,0x640000) or parsed.get('spiffs')!=(0xc90000,0x360000):
        raise ValueError('Unexpected partition layout')
    return manifest

def esptool_command(arguments):
    if getattr(sys,'frozen',False): return [sys.executable,'--esptool-internal',*arguments]
    return [sys.executable,'-m','esptool',*arguments]

def main():
    parser=argparse.ArgumentParser(description='First flash for an EMPTY ESP32-S3 N16R8. Default: offline checks only.')
    parser.add_argument('--bundle',type=pathlib.Path,default=root())
    parser.add_argument('--port')
    parser.add_argument('--list-ports',action='store_true')
    parser.add_argument('--blank-board',action='store_true')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if args.list_ports:
        from serial.tools import list_ports
        for port in list_ports.comports(): print(port.device,port.description)
        return
    folder=args.bundle.resolve(); manifest=verify_bundle(folder)
    print('PASS: four files, SHA256, ESP32-S3 image headers, addresses and partition boundaries')
    for entry in manifest['files']:print(entry['address'],entry['file'],entry['size'])
    if not args.execute:
        print('OFFLINE DRY RUN: device not opened. Empty-board startup remains untested.')
        return
    if not args.port or not args.blank_board:
        parser.error('Writing requires an explicit --port and --blank-board --execute')
    prefix=['--chip','esp32s3','--port',args.port,'--baud','460800','--after','no_reset']
    def run(tail):
        result=subprocess.run(esptool_command(prefix+tail),check=True,text=True,capture_output=True)
        print(result.stdout)
        return result.stdout
    chip=run(['chip_id'])
    if 'ESP32-S3' not in chip:raise ValueError('Unexpected device; refused')
    flash=run(['flash_id'])
    if not re.search(r'Detected flash size:\s*16MB',flash):raise ValueError('Flash is not 16MB; refused')
    security=run(['get_security_info'])
    if not re.search(r'Flags:\s*0x0+\b',security):raise ValueError('Security configuration is not known-empty; refused')
    with tempfile.TemporaryDirectory(prefix='codex-empty-check-') as temp:
        for start,length in [(0x9000,0x5000),(0xe000,0x2000),(0x10000,4096),(0x650000,4096),(0xc90000,4096)]:
            path=pathlib.Path(temp)/f'{start:x}.bin'
            run(['read_flash',hex(start),hex(length),str(path)])
            if any(byte!=255 for byte in path.read_bytes()):
                raise ValueError('Existing configuration/application detected. Use OTA or a deliberate recovery procedure.')
    addresses=[part for e in manifest['files'] for part in (e['address'],str(folder/e['file']))]
    run(['write_flash','--flash_mode','dio','--flash_freq','80m','--flash_size','16MB',*addresses])
    # esptool patches the bootloader header flash-size nibble during write. Match verification to that header.
    run(['verify_flash','--flash_mode','dio','--flash_freq','80m','--flash_size','16MB',*addresses])
    run(['run'])
    print('Flash contents verified. Check USB enumeration, version readback and a power-cycle before recording first boot as passed.')

if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--esptool-internal':
        import esptool
        esptool.main(sys.argv[2:])
    else:
        try:main()
        except (ValueError,OSError,subprocess.CalledProcessError) as error:
            print('STOP:',str(error),file=sys.stderr);sys.exit(1)
