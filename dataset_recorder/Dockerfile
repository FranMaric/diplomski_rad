FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-scipy \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-common-interfaces \
    && rm -rf /var/lib/apt/lists/*

COPY resample_30hz.py /scripts/resample_30hz.py
COPY add_metadata_topic.py /scripts/add_metadata_topic.py
COPY extract_contact_forces.py /scripts/extract_contact_forces.py

WORKDIR /data
