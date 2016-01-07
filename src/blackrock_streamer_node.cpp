#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blackrock_interface/blackrock_data.h"
#include "blackrock_interface/blackrock_channel_data.h"

#include <sstream>
#include "cbsdk.h"

#define INST 0
#define STREAM_FREQ 50

// Purpose: Test openning the library
cbSdkResult testOpen(void)
{
    cbSdkConnectionType conType = CBSDKCONNECTION_DEFAULT;

    // Library version can be read even before library open (return value is a warning)
    //  actual NSP version however needs library to be open
    cbSdkVersion ver;
    cbSdkResult res = cbSdkGetVersion(INST, &ver);

    printf("Initializing Cerebus real-time interface %d.%02d.%02d.%02d (protocol cb%d.%02d)...\n", ver.major, ver.minor, ver.release, ver.beta, ver.majorp, ver.minorp);

    cbSdkInstrumentType instType;
    res = cbSdkOpen(INST, conType);
    switch (res)
    {
    case CBSDKRESULT_SUCCESS:
        break;
    case CBSDKRESULT_NOTIMPLEMENTED:
        printf("Not implemented\n");
        break;
    case CBSDKRESULT_INVALIDPARAM:
        printf("Invalid parameter\n");
        break;
    case CBSDKRESULT_WARNOPEN:
        printf("Real-time interface already initialized\n");
    case CBSDKRESULT_ERROPENCENTRAL:
        printf("Unable to open library for Central interface\n");
        break;
    case CBSDKRESULT_ERROPENUDP:
        printf("Unable to open library for UDP interface\n");
        break;
    case CBSDKRESULT_ERROPENUDPPORT:
        res = cbSdkGetType(INST, NULL, &instType);
        if (instType == CBSDKINSTRUMENT_NPLAY || instType == CBSDKINSTRUMENT_REMOTENPLAY)
            printf("Unable to open UDP interface to nPlay\n");
        else
            printf("Unable to open UDP interface\n");
        break;
    case CBSDKRESULT_OPTERRUDP:
        printf("Unable to set UDP interface option\n");
        break;
    case CBSDKRESULT_MEMERRUDP:
        printf("Unable to assign UDP interface memory\n"
            " Consider using sysctl -w net.core.rmem_max=8388608\n"
            " or sysctl -w kern.ipc.maxsockbuf=8388608\n");
        break;
    case CBSDKRESULT_INVALIDINST:
        printf("Invalid UDP interface\n");
        break;
    case CBSDKRESULT_ERRMEMORYTRIAL:
        printf("Unable to allocate RAM for trial cache data\n");
        break;
    case CBSDKRESULT_ERROPENUDPTHREAD:
        printf("Unable to Create UDP thread\n");
        break;
    case CBSDKRESULT_ERROPENCENTRALTHREAD:
        printf("Unable to start Cerebus real-time data thread\n");
        break;
    case CBSDKRESULT_ERRINIT:
        printf("Library initialization error\n");
        break;
    case CBSDKRESULT_ERRMEMORY:
        printf("Library memory allocation error\n");
        break;
    case CBSDKRESULT_TIMEOUT:
        printf("Connection timeout error\n");
        break;
    case CBSDKRESULT_ERROFFLINE:
        printf("Instrument is offline\n");
        break;
    default:
        printf("Unexpected error\n");
        break;
    }

    if (res >= 0)
    {    
        // Return the actual openned connection
        res = cbSdkGetType(INST, &conType, &instType);
        if (res != CBSDKRESULT_SUCCESS)
            printf("Unable to determine connection type\n");
        res = cbSdkGetVersion(INST, &ver);
        if (res != CBSDKRESULT_SUCCESS)
            printf("Unable to determine instrument version\n");

        if (conType < 0 || conType > CBSDKCONNECTION_CLOSED)
            conType = CBSDKCONNECTION_COUNT;
        if (instType < 0 || instType > CBSDKINSTRUMENT_COUNT)
            instType = CBSDKINSTRUMENT_COUNT;
        
        char strConnection[CBSDKCONNECTION_COUNT + 1][8] = {"Default", "Central", "Udp", "Closed", "Unknown"};
        char strInstrument[CBSDKINSTRUMENT_COUNT + 1][13] = {"NSP", "nPlay", "Local NSP", "Remote nPlay", "Unknown"};
        printf("%s real-time interface to %s (%d.%02d.%02d.%02d) successfully initialized\n", strConnection[conType], strInstrument[instType], ver.nspmajor, ver.nspminor, ver.nsprelease, ver.nspbeta);
    }
    
    return res;
}

// Author & Date:   Ehsan Azar    25 Oct 2012
// Purpose: Test closing the library
cbSdkResult testClose(void)
{
    cbSdkResult res = cbSdkClose(INST);
    switch(res)
    {
    case CBSDKRESULT_SUCCESS:
        printf("Interface closed successfully\n");
        break;
    case CBSDKRESULT_WARNCLOSED:
        printf("Real-time interface already closed\n");
        break;
    default:
        printf("Unexpected error in closing the library!\n");
        break;
    }
    
    return res;
}

int main(int argc, char **argv)
{
  ros::Time::init();

  cbSdkResult res = testOpen();
  if (res < 0)
    ROS_INFO("testOpen failed (%d)!\n", res);
  else
    ROS_INFO("testOpen succeeded\n");

  UINT16 uBegChan   = 0;
  UINT32 uBegMask   = 0;
  UINT32 uBegVal    = 0;
  UINT16 uEndChan   = 0;
  UINT32 uEndMask   = 0;
  UINT32 uEndVal    = 0;
  bool   bDouble    = false;
  bool   bAbsolute  = false;
  UINT32 uWaveforms = 0;
  UINT32 uConts     = cbSdk_CONTINUOUS_DATA_SAMPLES;
  UINT32 uEvents    = cbSdk_EVENT_DATA_SAMPLES;
  UINT32 uComments  = 0;
  UINT32 uTrackings = 0;
  UINT32 bWithinTrial = false;

 // test cbSdkGet/Set Trial Config
  UINT32 bActive = 1;
  res = cbSdkSetTrialConfig(INST, bActive, uBegChan, uBegMask, uBegVal, uEndChan, uEndMask, uEndVal,
                         bDouble, uWaveforms, uConts, uEvents, uComments, uTrackings, bAbsolute);

  ros::Duration(.1).sleep();
  UINT32 bValid = 1;

  cbSdkTrialCont trialcont = cbSdkTrialCont();
  cbPKT_CHANINFO chan_info = cbPKT_CHANINFO();
  res = cbSdkInitTrialData(INST, NULL, &trialcont, NULL, NULL);
  int num_channels = trialcont.count;
  blackrock_interface::blackrock_data br_data_msg;
  br_data_msg.num_samples.resize(trialcont.count);
  br_data_msg.data.resize(trialcont.count);
  br_data_msg.channel_labels.resize(trialcont.count);
  for (int ii=0; ii<num_channels; ii++)
  {
    cbSdkGetChannelConfig(INST, trialcont.chan[ii], &chan_info); // Get a full channel configuration packet
    br_data_msg.channel_labels[ii].assign(chan_info.label);
  }

  ros::init(argc, argv, "br_data_stream");


  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<blackrock_interface::blackrock_data>("blackrock_data", 1);

  ros::Rate loop_rate(STREAM_FREQ);
  
  while (ros::ok())
  {
    // 1 - Get how many samples are waiting

    bool   bTrialDouble    = false;
    res = cbSdkGetTrialConfig(INST, NULL, NULL, NULL, NULL, NULL, NULL, NULL, &bTrialDouble, NULL, &uConts, &uEvents);

    res = cbSdkInitTrialData(INST, NULL, &trialcont, NULL, NULL);

    // 2 - Allocate buffer
    for (UINT32 channel = 0; channel < trialcont.count; channel++)
    {
        trialcont.samples[channel] = NULL;
        UINT32 num_samples = trialcont.num_samples[channel];
        if (bTrialDouble)
            trialcont.samples[channel] = calloc(num_samples,sizeof(double));
        else
            trialcont.samples[channel] = calloc(num_samples,sizeof(UINT16));
    }

    // 3 - get the data
    int bFlushBuffer = 1;
    INT16 **data = (INT16 **)trialcont.samples;
    res = cbSdkGetTrialData(INST, bFlushBuffer, NULL, &trialcont, NULL, NULL);
    data = (INT16 **)trialcont.samples;

    // 4 - forward the data
    blackrock_interface::blackrock_channel_data br_channel_data; // holds single chan data
    for (UINT32 channel = 0; channel < trialcont.count; channel++)
    {
      br_channel_data.channel_data.insert(
      br_channel_data.channel_data.begin(),
      data[channel],
      data[channel]+trialcont.num_samples[channel]);

      br_data_msg.data[channel] = br_channel_data;
      br_data_msg.num_samples[channel] = trialcont.num_samples[channel];
    
    }

    chatter_pub.publish(br_data_msg);

    loop_rate.sleep();
  }


  return 0;
}
