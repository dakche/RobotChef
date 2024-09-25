import os
import time

def delete_empty_files_and_matches(source_folder, target_folder):
    # Check if both folder paths are valid
    if not os.path.isdir(source_folder):
        print(f"The folder path '{source_folder}' is not valid.")
        return
    if not os.path.isdir(target_folder):
        print(f"The folder path '{target_folder}' is not valid.")
        return

    # List all files in the source folder
    for file_name in os.listdir(source_folder):
        img_name = file_name.replace('.txt', '')+'.jpg'

        source_file_path = os.path.join(source_folder, file_name)
        target_file_path = os.path.join(target_folder, img_name)

        # Check if the file is empty in the source folder
        if os.path.isfile(source_file_path) and os.path.getsize(source_file_path) == 0:
            try:
                # Delete the empty file from the source folder
                os.remove(source_file_path)
                print(f"Deleted empty file: {source_file_path}")

                # Check if a file with the same name exists in the target folder and delete it
                if os.path.isfile(target_file_path):
                    os.remove(target_file_path)
                    print(f"Deleted matching file in target folder: {target_file_path}")
            except Exception as e:
                print(f"Error deleting file {source_file_path} or {target_file_path}: {e}")

# Example usage:
source_folder = "valid/labels"  # Change this to your source folder path
target_folder = "valid/images"  # Change this to your target folder path
delete_empty_files_and_matches(source_folder, target_folder)
