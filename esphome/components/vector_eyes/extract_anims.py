import os
import tarfile
import glob
import shutil

# Paths
TAR_DIR = "/home/nonasuomy/code/esphome003/esphome/wire-os-externals/animation-assets/animations/"
OUTPUT_DIR = "/home/nonasuomy/code/esphome003/esphome/esphome/components/vector_eyes/animations_json/"

def extract_jsons():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    tar_files = glob.glob(os.path.join(TAR_DIR, "*.tar"))
    print(f"Found {len(tar_files)} tar files.")

    count = 0
    for tar_path in tar_files:
        try:
            with tarfile.open(tar_path, "r") as tar:
                for member in tar.getmembers():
                    if member.name.endswith(".json"):
                        # Extract to output dir
                        # We flatten the structure, so just extract the file
                        f = tar.extractfile(member)
                        if f:
                            out_path = os.path.join(OUTPUT_DIR, os.path.basename(member.name))
                            with open(out_path, "wb") as out_f:
                                shutil.copyfileobj(f, out_f)
                            print(f"Extracted: {member.name}")
                            count += 1
        except Exception as e:
            print(f"Error processing {tar_path}: {e}")

    print(f"Total extracted JSON files: {count}")

if __name__ == "__main__":
    extract_jsons()
