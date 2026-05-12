# Copyright 2025 Chair of Embedded Software (Computer Science 11) - RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import argparse
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, SequentialWriter, TopicMetadata
import rclpy
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from builtin_interfaces.msg import Time as TimeMsg


def trim_shift_timestamps(input_bag, output_bag, trim_seconds):
    rclpy.init() #initializing ros
    reader = SequentialReader() #opening rosbag2 in read-mode
    storage_options = StorageOptions(uri=input_bag, storage_id = 'sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options) 
    topics = reader.get_all_topics_and_types() #taking all topics in consideration
    first_timestamp = None #reading plus saving all messages and saving the first one
    stepValue = 1000
    print("Timestamp Mode started") #looking for first timestamp
    while reader.has_next():  
        (topic, data, timestamp) = reader.read_next()
        if first_timestamp is None:
            messages_info.append((topic, data, timestamp))
            first_timestamp = timestamp
            break
    if not messages_info:
        print("No messages found!")
        return
    trim_ns = trim_seconds * 1e9 #calculating the first timestamp we take into consideration
    cutoff_ns = first_timestamp + trim_ns
    print("cutting messages before timestamp " + str(int(cutoff_ns)) +".")
    reader.open(storage_options, converter_options) #opening reader and writer for the new bag
    writer = SequentialWriter()
    writer_storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer.open(writer_storage_options, converter_options)
    # topic_types = {t.name: t.type for t in topics} #saving messagetype for topics
    for topic in topics:
        writer.create_topic(topic)
    print("Writer Mode started")
    stepCounter = 0
    while reader.has_next(): #only take messages after timestamp into account
        (topic, data, timestamp) = reader.read_next()
        if timestamp < cutoff_ns:
            continue
        new_ros_time = int(timestamp - cutoff_ns) #writes message with timestamps starting at 0
        writer.write(topic, data, new_ros_time)
        if(stepCounter == stepValue):
            print("Write Mode")
            stepCounter = 0
        stepCounter = stepCounter + 1
    print("bag saved")
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="trimming start of ROS2 and shifts timestamps.")
    parser.add_argument('--input', required=True, help="Path to Input-Bag directory")
    parser.add_argument('--output', required=True, help="Path to Output-Bag directory")
    parser.add_argument('--trim', type=float, default=0.0, help="Time (in seconds) to be cut")
    
    args = parser.parse_args()
    trim_shift_timestamps(args.input, args.output, args.trim) 

if __name__ == "__main__":
    main()
