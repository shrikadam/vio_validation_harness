import sys

def nuke_sanitize(input_file, output_file):
    print(f"Aggressively cleaning: {input_file} -> {output_file}")
    valid_lines = 0
    
    with open(input_file, 'r', encoding='utf-8', errors='ignore') as fin, \
         open(output_file, 'w', encoding='utf-8') as fout:
        for line in fin:
            # 1. Strip ALL quotes, commas, and null bytes immediately
            line = line.replace('"', '').replace("'", '').replace(",", " ").replace('\x00', '')
            
            # Skip headers
            if not line.strip() or line.startswith('#') or 'timestamp' in line.lower():
                continue
            
            parts = line.split()
            
            if len(parts) >= 8:
                # 2. Join exactly 8 columns
                clean_line = " ".join(parts[:8])
                fout.write(clean_line + "\n")
                valid_lines += 1

    print(f"Saved {valid_lines} perfectly formatted lines.\n")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python convert_ov_to_tum.py <input.txt> <output.tum>")
    else:
        nuke_sanitize(sys.argv[1], sys.argv[2])