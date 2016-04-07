/*
 * This file is the unix multi-thread version of the LinearStackCompressor
 *
 * */

#ifndef THREADEDLINEARSTACKCOMPRESSOR_H
#define	THREADEDLINEARSTACKCOMPRESSOR_H

#include <stdio.h>
#include "ThreadedStaticBackgroundCompressor.h"
#include <queue>
#include <thread>
#include <mutex>
#include <thread>
#include <chrono>

class ThreadedLinearStackCompressor {
public:
    /* static const int headerSizeInBytes = 10240;
     *
     * the beginning of every file has a header headerSizeInBytes long; what is not used is
     * filled with 0s
     */
    static const int headerSizeInBytes = 10240;

    /* LinearStackCompressor();
     * 
     * on creation, the stack compressor is initialzed, but does not open a file
     */
    ThreadedLinearStackCompressor();

    /* virtual ~LinearStackCompressor();
     *
     * on destruction, the output file is closed and all pending frames are destroyed without being written
     *
     */
    virtual ~ThreadedLinearStackCompressor();

    /* virtual void newFrame(const IplImage *im, ImageMetaData *metadata = NULL);
     *
     * adds an image and any associated metadata to the stack of images to be compressed
     * may trigger adding the image to a static background compressor or
     * writing a static background image to disk, which takes time.
     * LSC will attempt to keep the compression time under 1/frameRate, but this is not
     * guaranteed behavior
     *
     * a copy of the IplImage is made and stored; the user is free to delete or reuse the image after calling
     * the metadata becomes the property and responsibility of the lsc;
     * the user should make no further use of the metadata and lsc will free it when appropriate
     */
    virtual void newFrame(const IplImage *im, ImageMetaData *metadata = NULL);

    /* virtual void setOutputFileName (const char *fname);
     * virtual void openOutputFile ();
     * virtual void closeOutputFile ();
     *
     * set the output file name prior to opening the output file
     * on openOutputFile, if an output file is already open,
     *   the already open file is closed first
     * closing the output file causes the header to be written but does not write any additional frames
     */
    virtual void setOutputFileName (const char *fname);
    virtual void openOutputFile ();
    virtual void closeOutputFile ();

    /* virtual void startRecording(int numFramesToRecord);
     *
     * virtual void startRecording(int numFramesToRecord);
     * tells the LSC to start adding frames to the stack until numFramesToRecord is reached
     * 
     */
    virtual void startRecording(int numFramesToRecord);

    /* virtual void stopRecording();
     * virtual void goIdle();
     *
     * synonyms;  stops adding frames to the stack and finishes compressing any that have
     * already been added.  puts the stack compressor in the default idle state where any incoming
     * frames are ignored
     */
    virtual void stopRecording();
    virtual void goIdle();

    /* virtual void startUpdatingBackground();
     *
     * puts the stack compressor in an upgraded idle state where the background is being computed but frames are not being added
     * to the stack.  pretty useless honestly
     */
    virtual void startUpdatingBackground();

    /* virtual inline void setThresholds(int threshBelowBackground, int threshAboveBackground, int smallDimMinSize, int lgDimMinSize) {
     *
     * threshBelowBackground: pixel values less than the background value must be this much below the background to count
     * threshAboveBackground: pixel values greater than the background value must be this much above the background to count
     * smallDimMinSize (s), lgDimMinSize (l): regions containing non-background pixels must be at least these dimensions (sxl or lxs)
     */
    virtual inline void setThresholds(int threshBelowBackground, int threshAboveBackground, int smallDimMinSize, int lgDimMinSize) {
        this->threshAboveBackground = threshAboveBackground;
        this->threshBelowBackground = threshBelowBackground;
        this->smallDimMinSize = smallDimMinSize;
        this->lgDimMinSize = lgDimMinSize;
    }
    /* virtual inline void setIntervals (int keyframeInterval, int backgroundUpdateInterval = 1)
     *
     * keyframe interval = how many frames go into one background subtracted stack with a common background, 90 seems to work well
     * backgroundUpdateInterval = how often to update the background in the stack compressor as frames are being added, default 1 is best
     *      unless frame rate is extraordinarily high
     */
    virtual inline void setIntervals (int keyframeInterval, int backgroundUpdateInterval = 1) {
        this->keyframeInterval = keyframeInterval;
        this->backgroundUpdateInterval = backgroundUpdateInterval;
    }
    virtual inline int getKeyFrameInterval() {
        return keyframeInterval;
    }
    /* virtual inline void setFrameRate (double frameRate)
     *
     * how often the lsc should expect frames to be arriving.  lsc attempts to keep function calls under this time, but no guarantees to work
     */
    virtual void setFrameRate (double frameRate) {
        this->frameRate = frameRate;
    }

    virtual std::ofstream::pos_type numBytesWritten ();

    virtual std::string saveDescription();

    virtual void numStacksWaiting (int &numToCompress, int &numToWrite);
    
    virtual inline int numFramesLeftToRecord() {
        return framesToRecord;
    }
    
protected:
    int keyframeInterval;
    int backgroundUpdateInterval;
    enum recordingState_t {idle, updatingBackground, recording};
    recordingState_t recordingState;
    int framesToRecord;
    std::ofstream *outfile;
    
    std::queue<ThreadedStaticBackgroundCompressor *> imageStacks; //stacks to compress
    std::queue<ThreadedStaticBackgroundCompressor *> compressedStacks; //stacks to merge
    std::queue<ThreadedStaticBackgroundCompressor *> stacksToWrite; //merged stacks to write
    
    
    ThreadedStaticBackgroundCompressor *activeStack;
    ThreadedStaticBackgroundCompressor *stackBeingCompressed;

    ThreadedStaticBackgroundCompressor *stackBeingWritten; //not used by lsc right now (?? MHG 6/6/13 - there's a function to set it)
    
    int numStacksToMerge;
    size_t memoryUsedByCompressedStacks;

    double frameRate;
    int threshBelowBackground;
    int threshAboveBackground;
    int smallDimMinSize;
    int lgDimMinSize;
    std::string fname;
    bool processing; //really should be a mutex, but whatever
//    bool lockActiveStack; //really should be a mutex, but whatever
//    std::mutex processing; //really should be a mutex, but whatever
    std::mutex lockActiveMutex; //really should be a mutex, but whatever
    std::mutex compressedStacksMutex_ ;
    std::mutex stacksToWriteMutex_;
    std::mutex imageStacksMutex_;

    std::string stacksavedescription;
    void mergeCompressedStacksThreadFcn();
    void startThreads();
    void stopThreads();
    std::thread mergeCompressedStacksThread_;
    bool mergeCompressedStacksActive_ ;

    bool compressionThreadActive_, writingThreadActive_;
    bool stacksLeftToCompress_ ;
    bool mergingStacks_ ;

    virtual void createStack();
    virtual void addFrameToStack(IplImage **im, ImageMetaData *metadata);
    virtual bool compressStack();
    virtual void mergeCompressedStacks(); // moves compressed stacks to compressedStacks vector, removes them from imageStacks
    virtual bool writeFinishedStack();
    virtual void setCompressionStack();
    virtual void setWritingStack(); //not used by lsc right now
    virtual void finishRecording ();
    
    virtual void writeHeader();
    virtual std::string headerDescription();
    virtual inline uint32_t idCode () {
        return 0xa3d2d45d; //CRC32 hash of "LinearStackCompressor" from fileformat.info
    }
    std::ofstream::pos_type currentFileSize;
private:
     ThreadedLinearStackCompressor(const ThreadedLinearStackCompressor& orig);
     void init();
};

#endif	/* LINEARSTACKCOMPRESSOR_H */

