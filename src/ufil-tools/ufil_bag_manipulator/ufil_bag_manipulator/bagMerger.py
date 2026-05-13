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
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, SequentialWriter
import rclpy


INPUT_BAG_1 = ""
INPUT_BAG_2 = ""
OUTPUT_BAG = "first_merged"

def message_generator(input_bag):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=input_bag, storage_id = 'sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next() #generating one message per step
        yield (topic, data, timestamp)

def merge_bags(input_bag_1, input_bag_2, output_bag):
    rclpy.init() #initializing ros
    genIn1 = message_generator(input_bag_1)
    genIn2 = message_generator(input_bag_2)

    message1 = next(genIn1, None)
    if message1 is not None:
        (topic1, data1, timestamp1) = message1
    message2 = next(genIn2, None)
    if message2 is not None:
        (topic2, data2, timestamp2) = message2
    reader = SequentialReader()
    storage_options_1 = StorageOptions(uri=input_bag_1, storage_id = 'sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options_1, converter_options)
    topics1 = reader.get_all_topics_and_types()
    #topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    storage_options_2 = StorageOptions(uri=input_bag_2, storage_id = 'sqlite3')
    reader.open(storage_options_2, converter_options)
    #topic_types.update({t.name: t.type for t in reader.get_all_topics_and_types()})
    topics2 = reader.get_all_topics_and_types()
    stepValue = 5000 #reading messages

    writer = SequentialWriter()
    writer_storage_options = StorageOptions(uri=output_bag, storage_id='sqlite3')
    writer.open(writer_storage_options, converter_options)
    created_topics = set()
    for topic in topics1: #adding all topics to bag
        if topic not in created_topics:
           writer.create_topic(topic)
           created_topics.add(topic)
    for topic in topics2: #adding all topics to bag
        if topic not in created_topics:
           writer.create_topic(topic)
           created_topics.add(topic)
    print("starting write mode")
    kept_count = 0 #writing all messages into bag
    count1 = 0
    count2 = 0
    while message1 is not None or message2 is not None:
        if message1 is not None and (message2 is None or timestamp1<= timestamp2):
            count1 += 1
            writer.write(topic1, data1, timestamp1)
            message1 = next(genIn1, None)
            if message1 is not None:
                (topic1, data1, timestamp1) = message1
        else: 
            count2 += 1
            writer.write(topic2, data2, timestamp2)
            message2 = next(genIn2, None)
            if message2 is not None:
                (topic2, data2, timestamp2) = message2
        kept_count += 1
        if(kept_count == stepValue):
            kept_count = 0
            print("write mode")
            print("Count1: " + str(int(count1)) + " Count2: " + str(int(count2)))
            count1 = 0
            count2 = 0
    print("bag saved")
    rclpy.shutdown()

def main():
    parser = argparse.ArgumentParser(description="merges two bags into one bag.")
    parser.add_argument('--input1', required=True, help="Path to Input1-Bag directory")
    parser.add_argument('--input2', required=True, help="Path to Input2-Bag directory")
    parser.add_argument('--output', required=True, help="Path to Output-Bag directory")
    
    args = parser.parse_args()
    merge_bags(args.input1, args.input2, args.output)

if __name__ == "__main__":
    main()

