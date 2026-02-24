# Extracts GPS data from ROS 2 bag files and exports it to CSV format.

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from geometry_msgs.msg import PoseStamped
from pathlib import Path
import csv

def extract_pose_data(bag_path, topic_name, output_file):
    # Configurar leitor do bag
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    # Escrever CSV
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['stamp_sec', 'stamp_nsec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == topic_name:
                msg = deserialize_message(data, PoseStamped)
                writer.writerow([msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                 msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    
    print(f"Dados de pose extraídos para: {output_file}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Extract GPS data from gps_fix message to CSV')
    parser.add_argument('bag_file', help='Path to folder that contains bag in .db3 format')
    parser.add_argument('--topic', default='/overhead/pose', help='Topic name to extract from bag')
    parser.add_argument('--output_data', default='irat_gt.csv', help='output file for pose data CSV')

    args = parser.parse_args()

    try:
        print("⏳ Starting pose data extraction...")
        extract_pose_data(
            args.bag_file,
            args.topic,
            args.output_data
        )
        print("✅ Extraction completed successfully!")
        print("📊 Generated file statistics:")
        print(f"   - Length: {Path(args.output_data).stat().st_size/1024:.2f} KB")
        print(f"   - Lines: {sum(1 for _ in open(args.output_data))}")

    except Exception as e:
        print(f"❌ Erro fatal: {str(e)}")