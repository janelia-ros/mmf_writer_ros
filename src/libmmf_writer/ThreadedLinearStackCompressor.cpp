/* 
 * File:   LinearStackCompressor.cpp
 * Author: Marc
 * 
 * Created on October 24, 2010, 3:29 PM
 *
 * (C) Marc Gershow; licensed under the Creative Commons Attribution Share Alike 3.0 United States License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/us/ or send a letter to
 * Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

#include "mmf_writer/ThreadedLinearStackCompressor.h"
#include "mmf_writer/Timer.h" // from marc's tic toc
#include <vector>
#include <string>
#include <sstream>
#include <queue>
using namespace std;
#include <ros/ros.h>

ThreadedLinearStackCompressor::ThreadedLinearStackCompressor() {
    init();
}

ThreadedLinearStackCompressor::ThreadedLinearStackCompressor(const ThreadedLinearStackCompressor& orig) {
}

ThreadedLinearStackCompressor::~ThreadedLinearStackCompressor() {
	   stopThreads();

	closeOutputFile();
   while (!imageStacks.empty()) {
       delete imageStacks.front();
       imageStacks.pop();
   }
   while (!compressedStacks.empty()) {
       delete compressedStacks.front();
       compressedStacks.pop();
   }
   while (!stacksToWrite.empty()) {
       delete stacksToWrite.front();
       stacksToWrite.pop();
   }
   if (activeStack != NULL) {
       delete activeStack;
   }


}


void ThreadedLinearStackCompressor::init() {
    keyframeInterval = 64;
    backgroundUpdateInterval = 1;
    recordingState = idle;
    framesToRecord = 0;
    outfile = NULL;
    activeStack = NULL;
    stackBeingCompressed = NULL;
    stackBeingWritten = NULL;
    frameRate = 30;
    threshBelowBackground = 5;
    threshAboveBackground = 5;
    lgDimMinSize = 2;
    smallDimMinSize = 3;
    processing = false; //really should be a mutex, but whatever
//    lockActiveStack = false; //really should be a mutex, but whatever
    currentFileSize_ = 0;
    numStacksToMerge = 12; // Was originally 12
    memoryUsedByCompressedStacks = 0;
    // thread stuff
    compressionThreadActive_ = false;
    writeThreadActive_ = false;
    savingStacks_ = false;

}

void ThreadedLinearStackCompressor::newFrame(const IplImage* im, ImageMetaData *metadata) {
    int maxCycles = (int) 1E9; //just so it doesn't hang

    Timer tim = Timer();
    tim.start();

    IplImage *imcpy = cvCloneImage(im);

    lockActiveMutex.lock();
    addFrameToStack(&imcpy, metadata);
    lockActiveMutex.unlock();
//    lockActiveStack = false;

    if (recordingState == recording ) {
        if(--framesToRecord <= 0 ) {
        	finishRecording();
        }
    }



    // ok this will not work as it is since we need to check if all the images are there to start processing the stack!
    //std::ostringstream fw ;
    //fw << "frames waiting to process: " << activeStack->framesWaitingToProcess() ;
    //ROS_WARN(fw.str().c_str());


    // This is where multi-threading would be useful :-)
    // Adding frames to stack is not worrisome
    if (!processing) {
        processing = true;
        while (tim.getElapsedTimeInSec() < 0.95/frameRate && compressStack()) {
        //intentionally blank
        }
//        while (tim.getElapsedTimeInSec() < 0.95/frameRate && writeFinishedStack()) {
            //intentionally blank
//        }
        processing = false;
    }
}

void ThreadedLinearStackCompressor::addFrameToStack(IplImage **im, ImageMetaData *metadata) {
    if (activeStack == NULL) {
        createStack();
    }
    // After keyframeInterval, we add the stack of imgs to the imageStacks so it can start to be processed
    if (activeStack->numToProccess() >= keyframeInterval) {
        imageStacks.push(activeStack);        
        createStack();
    }
    if (recordingState == recording) {
        activeStack->addFrame(im, metadata);
    } else {
        if (recordingState == updatingBackground) {
            activeStack->updateBackground(*im);            
        }
        if (metadata != NULL) {
            delete metadata;
        }
        cvReleaseImage(im);
    }
}

void ThreadedLinearStackCompressor::startRecording(int numFramesToRecord) {

	startThreads();

	recordingState = recording;
    this->framesToRecord = numFramesToRecord;

}

void ThreadedLinearStackCompressor::finishRecording() {
    ROS_INFO("We enter the finishrecording()");

    processing = true;
    lockActiveMutex.lock();

    ROS_INFO("We set recording state to idle");
    recordingState = idle;
    ROS_INFO("We push the active stack to imageStacks");

    if (activeStack != NULL) {
        if (activeStack->numToProccess() > 0) {
            imageStacks.push(activeStack);
        } else {
            delete activeStack;
        }
        activeStack = NULL;
    }
    lockActiveMutex.unlock();

    ROS_INFO("We get to the finishrecording->compressStack part");
    while (compressStack()) {
        //intentionally blank
    }


    ROS_INFO("We enter the writeFinishedStack, this should be a blocking function");
    writeFinishedStack();
    ROS_INFO("We are done with the writeFinishedStack");

    //    while (writeFinishedStack()) {
//        //intentionally blank
//    }
   

    processing = false;
    stopThreads();
    //    lockActiveStack = false;
    ROS_INFO("We exit finishrecording inside lsc");
}

void ThreadedLinearStackCompressor::stopThreads() {
	mergeCompressedStacksThreadActive_ = false;
	if (mergeCompressedStacksThread_.joinable()) {
		mergeCompressedStacksThread_.join();
	}
	writeThreadActive_ = false;
	if (writeThread_.joinable()) {
		writeThread_.join();
	}

}

void ThreadedLinearStackCompressor::createStack() {
    activeStack = new ThreadedStaticBackgroundCompressor();
    activeStack->setAutomaticUpdateInterval(backgroundUpdateInterval);
    activeStack->setThresholds(threshBelowBackground, threshAboveBackground, smallDimMinSize, lgDimMinSize);
}

//returns true if there may be images remaining to compress
bool ThreadedLinearStackCompressor::compressStack() {
     setCompressionStack();
     if (stackBeingCompressed != NULL) {
    	 // In my linux implementation, once we call this function, a # of threads processes all
    	 // the images in sbc , so we don't need to "threadify" this part of the code.
    	 // The windows version has this on a thread in the linear stack compressor
    	 stackBeingCompressed->processFrame();
         return true;
     }
     return false;
}



void ThreadedLinearStackCompressor::numStacksWaiting(int& numToCompress, int& numToWrite) {
    
    numToCompress = imageStacks.size();
    numToWrite = compressedStacks.size() + stacksToWrite.size();
    
}

void ThreadedLinearStackCompressor::setCompressionStack() {
   std::lock_guard<std::mutex> lock(compressedStacksMutex_);

   //ROS_INFO("We enter setCompressionStack");
	// Checks if we're done compressing the stack?
	// This gets triggered if we are done processing the stack
	// we check if there is a stack being compressed and if the numToProcess has reached 0, ie, it ended
   if (stackBeingCompressed != NULL && stackBeingCompressed->numToProccess() <= 0) {
	   // we put a new compressed stack in the compressedStacks vector
	   ROS_INFO("We put a new compressed stack in the vector");
	   compressedStacks.push(stackBeingCompressed);
        memoryUsedByCompressedStacks += stackBeingCompressed->sizeInMemory();
        stackBeingCompressed = NULL;
    }
   // We add a new stack to be compressed if there's one in the imageStacks queue, this only happens when the keyframe value is reached
   std::lock_guard<std::mutex> lockimgStack(imageStacksMutex_);
   if (stackBeingCompressed == NULL && !imageStacks.empty()) {
	   ROS_INFO("We get a new stack from imagestacks into stackBeing compressed");
	   stackBeingCompressed = imageStacks.front();
        imageStacks.pop();
    }

   stacksLeftToCompress_ = (stackBeingCompressed != NULL);
}

//returns true if there may be stacks remaining to write
bool ThreadedLinearStackCompressor::writeFinishedStack() {

//	mergeCompressedStacks();

	ROS_INFO(" We wait until out thread goes through our compressedStacks vector");

	ROS_INFO("compressedStacks.empty is %d", compressedStacks.empty());
	ROS_INFO("imageStacks.empty is %d", imageStacks.empty());
	ROS_INFO("mergingStacks_ is %d", mergingStacks_);
	while( !compressedStacks.empty() || !imageStacks.empty() || mergingStacks_) {
		setCompressionStack();
		ROS_INFO("compressedStacks.empty is %d", compressedStacks.empty());
		ROS_INFO("imageStacks.empty is %d", imageStacks.empty());
		ROS_INFO("mergingStacks_ is %d", mergingStacks_);

		std::this_thread::sleep_for (std::chrono::milliseconds(250));
		// intentionally left blank
		}

	ROS_INFO("All merged, no more compressed stacks!");

	ROS_INFO("lsc->writeFinished->setWritingStack!");

	// We wait until we have finished writing all stacks;
	while(stacksLeftToWrite_ > 0 || savingStacks_ ) {
		std::this_thread::sleep_for (std::chrono::milliseconds(250));
	}
	ROS_INFO("We have no more stacks left to write!");

	return true;

}
    

ofstream::pos_type ThreadedLinearStackCompressor::numBytesWritten() {

    return currentFileSize_;
}


void ThreadedLinearStackCompressor::openOutputFile() {
    if (outfile != NULL) {
        ThreadedLinearStackCompressor::closeOutputFile();
    }
    if (!fname.empty()) {
        outfile = new ofstream (fname.c_str(),ofstream::binary);
        writeHeader();
    }
}

void ThreadedLinearStackCompressor::closeOutputFile() {
     if (outfile != NULL) {
        outfile->seekp(0);
        writeHeader(); //now that we have an updated save description
        outfile->close();
        delete outfile;
        outfile = NULL;
    }
}

void ThreadedLinearStackCompressor::setOutputFileName(const char* fname) {
    if (fname != NULL) {
        this->fname = string(fname);
    }
}

void ThreadedLinearStackCompressor::stopRecording() {
    //recordingState = idle;
    finishRecording();
}

void ThreadedLinearStackCompressor::goIdle() {
    stopRecording();
}

void ThreadedLinearStackCompressor::startUpdatingBackground() {
    recordingState = updatingBackground;
}

void ThreadedLinearStackCompressor::writeHeader() {
    if (outfile == NULL) {
        return;
    }
    char zero[headerSizeInBytes] = {0};
    ofstream::pos_type cloc = outfile->tellp();
    outfile->write(zero, headerSizeInBytes);
    ofstream::pos_type eloc = outfile->tellp();
    outfile->seekp(cloc);

    string sd = saveDescription();
    outfile->write(sd.c_str(), sd.length() + 1);
    uint32_t idcode = idCode();
    outfile->write((char *) &idcode, sizeof(idcode));
    int info[10] = {0};
    info[0] = headerSizeInBytes;
    info[1] = keyframeInterval;
    info[2] = threshBelowBackground;
    info[3] = threshAboveBackground;
    outfile->write((char *) info, sizeof(info));
    outfile->seekp(eloc);

}

string ThreadedLinearStackCompressor::headerDescription() {
    stringstream os;
    os << headerSizeInBytes << " byte zero padded header beginning with a textual description of the file, followed by \\0 then the following fields (all ints, except idcode)\n";
    os << sizeof(uint32_t) << " byte uint32_t idcode = " << hex << idCode() << dec << ", header size in bytes, key frame interval, threshold below background, threshold above background\n";
    return os.str();
}

string ThreadedLinearStackCompressor::saveDescription() {
    stringstream os;
    os << "Set of Image Stacks representing a movie. Beginning of file is a header, with this format:\n" << headerDescription();
    os << "Header is followed by a set of common background image stacks, with the following format:\n" << stacksavedescription;
    return os.str();
}

void ThreadedLinearStackCompressor::startThreads() {
	mergeCompressedStacksThreadActive_ = true;
	mergeCompressedStacksThread_ = 	std::thread(&ThreadedLinearStackCompressor::mergeCompressedStacksThreadFcn, this);
	writeThreadActive_ = true ;
	writeThread_ = std::thread(&ThreadedLinearStackCompressor::writeThreadFcn, this);

}

void ThreadedLinearStackCompressor::writeThreadFcn() {
	ROS_INFO("We start out writing thread");
	while (writeThreadActive_) {

		setWritingStack();

		if (stacksLeftToWrite_) {
			savingStacks_ = true;
			ROS_INFO("Writing stack!");
			stackBeingWrittenMutex_.lock();
			if (stacksavedescription.empty()) {
				//			                writingThreadTimer.tic("creating save description");
				stacksavedescription = stackBeingWritten->saveDescription();
				//			                writingThreadTimer.toc("creating save description");
				ROS_INFO("created save description");
			}

			outfileMutex_.lock();
            if (outfile == NULL) {
            	ROS_INFO("We open the file!");
            	openOutputFile();
            }
            if (outfile == NULL) {
                ROS_ERROR("error opening output file");

            }

            stackBeingWritten->toDisk(*outfile);
            currentFileSize_ = outfile->tellp();
            ROS_INFO("We wrote to the disk");
            outfileMutex_.unlock();

            delete stackBeingWritten;
            stackBeingWritten= NULL;
            stackBeingWrittenMutex_.unlock();
            savingStacks_ = false;
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

	}

	ROS_INFO("We exit our writing thread");
}

void ThreadedLinearStackCompressor::mergeCompressedStacksThreadFcn() {

	while (mergeCompressedStacksThreadActive_) {

		setCompressionStack();
		compressedStacksMutex_.lock();

	    if ((recordingState != recording && !compressedStacks.empty()) || compressedStacks.size() >= numStacksToMerge) {
	        ROS_INFO( "merging stacks in the thread function ");
	        mergingStacks_ = true;
	        ThreadedStaticBackgroundCompressor *sbc = compressedStacks.front();
	        compressedStacks.pop();
	        vector<ThreadedStaticBackgroundCompressor *> stacksToMerge;
	        for (int j = 1; !compressedStacks.empty() && j < numStacksToMerge; ++j) {
	            stacksToMerge.push_back(compressedStacks.front());
	            compressedStacks.pop();
	        }
	        compressedStacksMutex_.unlock();
	        memoryUsedByCompressedStacks *= (compressedStacks.size() / (compressedStacks.size() + numStacksToMerge)); //estimates, in the unusual case compressedStacks wasn't emptied

	        sbc->mergeStacks(stacksToMerge);
	        for (vector<ThreadedStaticBackgroundCompressor *>::iterator it = stacksToMerge.begin(); it != stacksToMerge.end(); ++it) {
	            delete (*it);
	            *it = NULL;
	        }
	        stacksToWriteMutex_.lock();
	        stacksToWrite.push(sbc);
	        stacksToWriteMutex_.unlock();

	        ROS_INFO("done merging stacks in the thread");
	        mergingStacks_ = false;

	    }

	    compressedStacksMutex_.unlock();

	    std::this_thread::sleep_for(std::chrono::milliseconds(100));

	}

}

void ThreadedLinearStackCompressor::mergeCompressedStacks() {
	return; // We now use a thread just for this :)

	std::lock_guard<std::mutex> lock(compressedStacksMutex_);
    
    if ((recordingState != recording && !compressedStacks.empty()) || compressedStacks.size() >= numStacksToMerge) {
        ROS_INFO("merging stacks in the function");
        ThreadedStaticBackgroundCompressor *sbc = compressedStacks.front();
        // let's do some debug
//        std::stringstream os;
//        for (int b=0; b < sbc->numProcessed(); b++) {
//        	const ImageMetaData * md = sbc->getMetaData(b);
//        	std::map<std::string, double> gimme = md->getFieldNamesAndValues();
//        	os << "bufnum: " << gimme["bufnum"] << ", bufnum_time: " << gimme["bufnum_time"] << std::endl ;
//        }
//    	ROS_INFO(os.str().c_str());
        compressedStacks.pop();
        vector<ThreadedStaticBackgroundCompressor *> stacksToMerge;
        for (int j = 1; !compressedStacks.empty() && j < numStacksToMerge; ++j) {
            stacksToMerge.push_back(compressedStacks.front());
            compressedStacks.pop();
        }
        memoryUsedByCompressedStacks *= (compressedStacks.size() / (compressedStacks.size() + numStacksToMerge)); //estimates, in the unusual case compressedStacks wasn't emptied

        sbc->mergeStacks(stacksToMerge);
        for (vector<ThreadedStaticBackgroundCompressor *>::iterator it = stacksToMerge.begin(); it != stacksToMerge.end(); ++it) {
            delete (*it);
            *it = NULL;
        }      
        stacksToWrite.push(sbc);
        cout << "done merging stacks " << endl << flush;
    }
}


void ThreadedLinearStackCompressor::setWritingStack() {
     std::lock_guard<std::mutex> lock(stacksToWriteMutex_);
     std::lock_guard<std::mutex> lock2(stackBeingWrittenMutex_);
     
    if (stackBeingWritten == NULL && !stacksToWrite.empty()) {
        stackBeingWritten = stacksToWrite.front();
        stacksToWrite.pop();
    }
    
    stacksLeftToWrite_ = (stackBeingWritten != NULL);

}
