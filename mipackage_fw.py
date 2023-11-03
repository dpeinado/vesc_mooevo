import shutil
import os
import subprocess

# https://stackoverflow.com/questions/14989858/get-the-current-git-hash-in-a-python-script
def get_git_revision_short_hash() -> str:
    return subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()

# Get the origin and destionation directories
build_dir = os.path.dirname(os.path.abspath(__file__)) + '/build'
package_dir = os.path.dirname(os.path.abspath(__file__)) + '/package'

# Get the short git hash
git_hash = get_git_revision_short_hash()

# Define default destination filenames
no_limits_name = "VESC_default_no_hw_limits.bin"
default_name = "VESC_default.bin"

# Add directories and targets to the dictionary
# package_dict["group name diplayed in firmware tab of the vesc tool"] = [['.c filename minus the hw_', 'compiled .bin filename']]
package_dict = {}
package_dict["60"] = [['60', default_name]]
package_dict["60_mooevo"] = [['60_mooevo', default_name]]

# This is the firmware stub string
res_firmwares_string = '        <file>TARGET_DESTINATION_DIRECTORY/TARGET_DESTINATION_FILENAME</file>\n'

# This is the XML stub string
resource_xml_stub_string = '''
<RCC>
   <qresource prefix="/res/firmwares/">
REPLACEABLE_STRING
   </qresource>
</RCC>
'''

# Declare an empty string
res_string = ""

# Iterate over all directories in the dictionary
for directory in package_dict:

    # Set the destination path
    destination_path = os.path.join(package_dir, directory)

    # Create the destination directory
    os.makedirs(destination_path, exist_ok=True)

    # Iterate over each target
    for target in package_dict[directory]:
        # Shorthand variable
        destination_file_name = target[1]
        origin_file_name = target[0] + '.bin'

        # Copy the file
        shutil.copy(os.path.join(build_dir, target[0], origin_file_name), os.path.join(destination_path, destination_file_name))

        # Replace the stub string with the target specifics
        target_res_string = res_firmwares_string.replace("TARGET_DESTINATION_DIRECTORY", directory).replace("TARGET_DESTINATION_FILENAME", destination_file_name)

        # Add this string to the master Qt resource string
        res_string = res_string + target_res_string

# Print the QRC file
with open(os.path.join(package_dir, 'res_fw.qrc'), 'w') as f:
    print(resource_xml_stub_string.replace("REPLACEABLE_STRING", res_string[:-1]), file=f)

