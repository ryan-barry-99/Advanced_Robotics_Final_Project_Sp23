import pandas as pd
import ast
import os

balloon_data = 'Balloon_Dataset/balloon-data.csv'
# Load the annotations file into a pandas dataframe
balloons_df = pd.read_csv(balloon_data)

# Create the class names file
with open('obj.names', 'w') as f:
    f.write('balloon\n')

# Create the directory for the annotation files
os.makedirs('Balloon_Dataset/annotations', exist_ok=True)

# Create a directory for the YOLO annotations if it doesn't already exist
os.makedirs('Balloon_Dataset/yolo_annotations', exist_ok=True)

# Create the annotation files for the training set
for index, row in balloons_df.iterrows():
    filename = row['fname']
    num_balloons = row['num_balloons']
    bboxes = ast.literal_eval(row['bbox'])

    with open(f'Balloon_Dataset/yolo_annotations/{os.path.splitext(filename)[0]}.txt', 'w') as f:
        for bbox in bboxes:
            xmin = bbox['xmin']
            ymin = bbox['ymin']
            xmax = bbox['xmax']
            ymax = bbox['ymax']

            x_center = (xmin + xmax) / 2 / row['width']
            y_center = (ymin + ymax) / 2 / row['height']
            width = (xmax - xmin) / row['width']
            height = (ymax - ymin) / row['height']

            f.write(f'0 {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n')