# ros_blackrock_streamer

Uses cbsdk library and other code from https://github.com/dashesy/CereLink to stream blackrock data into ROS.

Streams data at a fixed 50Hz rate.

To run, first run central on a windows computer and set up the channels you want to stream

Then, on the linux computer this is running on, run: sudo sysctl -w net.core.rmem_max=8388608
and set the ethernet network to be 192.168.137.1
then run roscore
then rosrun blackrock_streamer (or ./devel/lib/blackrock_interface/blackrock_streamer if the package isn't in your ros path)

to check it is streaming, you can run rostopic echo /blackrock_data
you can also plot it, e.g. rqt_plot /blackrock_data/data[0]/channel_data[1] will plot the first channel's 2nd datapoing from each incoming packet (should be ~20 datapoints per packet when running this with blackrock set at 1000Hz).
