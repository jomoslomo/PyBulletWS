import os

def parse_text_file(file_path):
    """Parse the text file and extract link and joint information."""
    links = []
    joints = []
    current_section = None
    current_data = {}

    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            
            if line.startswith('[') and line.endswith(']'):
                # Save the previous section's data
                if current_section == 'Link' and current_data:
                    links.append(current_data)
                elif current_section == 'Joint' and current_data:
                    joints.append(current_data)
                
                # Start a new section
                current_section = line[1:-1]
                current_data = {}
            else:
                # Read the key-value pairs
                key, value = line.split(':', 1)
                current_data[key.strip()] = value.strip()

        # Add the last section
        if current_section == 'Link' and current_data:
            links.append(current_data)
        elif current_section == 'Joint' and current_data:
            joints.append(current_data)

    return links, joints

def generate_urdf(links, joints, output_file):
    """Generate a URDF file based on the parsed data."""
    with open(output_file, 'w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write('<robot name="robot_v4">\n\n')

        # Generate links
        for link in links:
            f.write(f'    <!-- Link: {link["name"]} -->\n')
            f.write(f'    <link name="{link["name"]}">\n')
            f.write('        <inertial>\n')
            origin_params = link["origin"].split()
            f.write(f'            <origin xyz="{origin_params[0]} {origin_params[1]} {origin_params[2]}" rpy="{origin_params[3]} {origin_params[4]} {origin_params[5]}"/>\n')
            f.write(f'            <mass value="{link["mass"]}"/>\n')
            inertia = link["inertia"].split()
            f.write(f'            <inertia ixx="{inertia[0]}" ixy="{inertia[1]}" ixz="{inertia[2]}" iyy="{inertia[3]}" iyz="{inertia[4]}" izz="{inertia[5]}"/>\n')
            f.write('        </inertial>\n')
            f.write('        <visual>\n')
            visual_params = link["visual"].split()
            f.write(f'            <origin xyz="{visual_params[1]} {visual_params[2]} {visual_params[3]}" rpy="{visual_params[4]} {visual_params[5]} {visual_params[6]}"/>\n')
            f.write(f'            <geometry>\n')
            f.write(f'                <mesh filename="{visual_params[0]}" scale="{visual_params[7]} {visual_params[8]} {visual_params[9]}"/>\n')
            f.write('            </geometry>\n')
            f.write(f'            <material name="{visual_params[10]}">\n')
            f.write(f'                <color rgba="{visual_params[11]} {visual_params[12]} {visual_params[13]} {visual_params[14]}"/>\n')
            f.write('            </material>\n')
            f.write('        </visual>\n')
            f.write('        <collision>\n')
            collision_params = link["collision"].split()
            f.write(f'            <origin xyz="{collision_params[1]} {collision_params[2]} {collision_params[3]}" rpy="{collision_params[4]} {collision_params[5]} {collision_params[6]}"/>\n')
            f.write(f'            <geometry>\n')
            f.write(f'                <mesh filename="{collision_params[0]}" scale="{collision_params[7]} {collision_params[8]} {collision_params[9]}"/>\n')
            f.write('            </geometry>\n')
            f.write('        </collision>\n')
            f.write('    </link>\n\n')

        # Generate joints
        for joint in joints:
            f.write(f'    <!-- Joint: {joint["name"]} -->\n')
            f.write(f'    <joint name="{joint["name"]}" type="{joint["type"]}">\n')
            f.write(f'        <parent link="{joint["parent"]}"/>\n')
            f.write(f'        <child link="{joint["child"]}"/>\n')
            origin_params = joint["origin"].split()
            f.write(f'        <origin xyz="{origin_params[0]} {origin_params[1]} {origin_params[2]}" rpy="{origin_params[3]} {origin_params[4]} {origin_params[5]}"/>\n')
            
            # Write axis if it exists
            if "axis" in joint:
                f.write(f'        <axis xyz="{joint["axis"]}"/>\n')
            
            # Write limit if it exists
            if "limit" in joint:
                limit = joint["limit"].split()
                f.write(f'        <limit lower="{limit[0]}" upper="{limit[1]}" effort="{limit[2]}" velocity="{limit[3]}"/>\n')

            # Write dynamics if it exists
            if "dynamics" in joint:
                dynamics = joint["dynamics"].split()
                f.write(f'        <dynamics damping="{dynamics[0]}" friction="{dynamics[1]}"/>\n')

            # Write visual if it exists
            if "visual" in joint:
                f.write('        <visual>\n')
                visual_params = joint["visual"].split()
                f.write(f'            <origin xyz="{visual_params[1]} {visual_params[2]} {visual_params[3]}" rpy="{visual_params[4]} {visual_params[5]} {visual_params[6]}"/>\n')
                f.write(f'            <geometry>\n')
                f.write(f'                <mesh filename="{visual_params[0]}" scale="{visual_params[7]} {visual_params[8]} {visual_params[9]}"/>\n')
                f.write('            </geometry>\n')
                f.write(f'            <material name="{visual_params[10]}">\n')
                f.write(f'                <color rgba="{visual_params[11]} {visual_params[12]} {visual_params[13]} {visual_params[14]}"/>\n')
                f.write('            </material>\n')
                f.write('        </visual>\n')

            # Write collision if it exists
            if "collision" in joint:
                f.write('        <collision>\n')
                collision_params = joint["collision"].split()
                f.write(f'            <origin xyz="{collision_params[1]} {collision_params[2]} {collision_params[3]}" rpy="{collision_params[4]} {collision_params[5]} {collision_params[6]}"/>\n')
                f.write(f'            <geometry>\n')
                f.write(f'                <mesh filename="{collision_params[0]}" scale="{collision_params[7]} {collision_params[8]} {collision_params[9]}"/>\n')
                f.write('            </geometry>\n')
                f.write('        </collision>\n')

            f.write('    </joint>\n\n')

        f.write('</robot>\n')

def main():
    # Define input and output files
    input_file = "ur.txt"
    output_file = "robot.urdf"
    
    # Parse the text file
    links, joints = parse_text_file(input_file)
    
    # Generate URDF file
    generate_urdf(links, joints, output_file)
    print(f"URDF file generated: {output_file}")

if __name__ == "__main__":
    main()
