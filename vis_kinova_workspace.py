import pandas as pd

def create_mjcf_circle(objects_list):
    xml_template = '''<?xml version="1.0" encoding="utf-8"?>
    <mujoco>
        <worldbody>
            {}
        </worldbody>
    </mujoco>'''

    object_template = '''<body name="object_{}" pos="{} {} {}">
                          <geom type="sphere" size="{}" rgba="{} {} {} 0.5" />
                        </body>'''

    object_xml_list = []

    for idx, point in enumerate(objects_list):
        # print(point)
        x, y, z,x_normalized,y_normalized,z_normalized = point

        object_xml_list.append(object_template.format(idx, x, y, z, 0.01,x_normalized,y_normalized,z_normalized))

    mjcf_xml = xml_template.format("\n".join(object_xml_list))
    return mjcf_xml

def getPartWorkSpace(data):
        
    data = data[data['x'] >= 0.05]
    data = data[data['y'] < 0]

    arm_left = data[data['y'] >= -0.144]

    arm_left = arm_left.sample(2000)
    arm_right = data[data['y'] < -0.144]

    result = pd.concat([arm_left, arm_right], ignore_index=True)

    # res  = arm_left+arm_right
    dt = pd.DataFrame(result)
    dt.to_csv('workspace_part_A.csv',index=False)

def load_data_from_csv(csv_file, num_samples):
    df = pd.read_csv(csv_file)
    df = df[df['x'] <1]
    df = df[df['x'] < 1]
    df = df[df['z'] < 1]
    df = df[df['z'] >0.2]

    # prsint(df)
    x = df['x']
    y = df['y']
    z = df['z']
    x_normalized = (x - x.min()) / (x.max() - x.min())
    y_normalized = (y - y.min()) / (y.max() - y.min())
    z_normalized = (z - z.min()) / (z.max() - z.min())
    df['x_normalized'] = x_normalized
    df['y_normalized'] = y_normalized
    df['z_normalized'] = z_normalized

    res = df.sample(num_samples)
    res.to_csv('kinova_workspaceSample.csv',index=False)

    return res.values.tolist()

csv_file_path = "./kinova_workspace.csv"
num_samples = 100
data_points = load_data_from_csv(csv_file_path, num_samples)
mjcf_xml_content = create_mjcf_circle(data_points)

with open("./objects.xml", "w") as f:
    f.write(mjcf_xml_content)
