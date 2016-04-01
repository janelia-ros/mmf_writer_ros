/* 
 * File:   StaticBackgroundCompressor.cpp
 * Author: Marc
 * 
 * Created on October 4, 2010, 1:06 PM
 *
 * (C) Marc Gershow; licensed under the Creative Commons Attribution Share Alike 3.0 United States License.
 * To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/3.0/us/ or send a letter to
 * Creative Commons, 171 Second Street, Suite 300, San Francisco, California, 94105, USA.
 *
 */

//#include "highgui.h"

#include "mmf_writer/ThreadedStaticBackgroundCompressor.h"
#include <vector>
#include "mmf_writer/BackgroundRemovedImage.h"
#include "cv.h"
//#include <opencv2/
#include <opencv2/highgui/highgui.hpp>

//#include "highgui.h"
#include "mmf_writer/BackgroundRemovedImageLoader.h"
#include "mmf_writer/StackReader.h"
#include "mmf_writer/IplImageLoaderFixedWidth.h"
#include <iostream>
#include <fstream>
#include <sstream>

// using Boost's thread instead of C++11 since ROS depends on it
// c++11 std::thread would have been maybe more universal :)
//#include <boost/asio/io_service.hpp>
//#include <boost/bind.hpp>
//#include <boost/thread/thread.hpp>
//
//#include <mmf_writer/ThreadPool.h>
#include <mmf_writer/Timer.h>
#include <ros/ros.h>


typedef struct __attribute__((packed,aligned(4))) _IplImage136
{
    int  nSize;             /* sizeof(IplImage) */
    int  ID;                /* version (=0)*/
    int  nChannels;         /* Most of OpenCV functions support 1,2,3 or 4 channels */
    int  alphaChannel;      /* Ignored by OpenCV */
    int  depth;             /* Pixel depth in bits: IPL_DEPTH_8U, IPL_DEPTH_8S, IPL_DEPTH_16S,
                               IPL_DEPTH_32S, IPL_DEPTH_32F and IPL_DEPTH_64F are supported.  */
    char colorModel[4];     /* Ignored by OpenCV */
    char channelSeq[4];     /* ditto */
    int  dataOrder;         /* 0 - interleaved color channels, 1 - separate color channels.
                               cvCreateImage can only create interleaved images */
    int  origin;            /* 0 - top-left origin,
                               1 - bottom-left origin (Windows bitmaps style).  */
    int  align;             /* Alignment of image rows (4 or 8).
                               OpenCV ignores it and uses widthStep instead.    */
    int  width;             /* Image width in pixels.                           */
    int  height;            /* Image height in pixels.                          */
    struct _IplROI *roi;    /* Image ROI. If NULL, the whole image is selected. */
    struct _IplImage *maskROI;      /* Must be NULL. */
    void  *imageId;                 /* "           " */
    struct _IplTileInfo *tileInfo;  /* "           " */
    int  imageSize;         /* Image data size in bytes
                               (==image->height*image->widthStep
                               in case of interleaved data)*/
    char *imageData;        /* Pointer to aligned image data.         */
    int  widthStep;         /* Size of aligned image row in bytes.    */
    int  BorderMode[4];     /* Ignored by OpenCV.                     */
    int  BorderConst[4];    /* Ditto.                                 */
    char *imageDataOrigin;  /* Pointer to very origin of image data
                               (not necessarily aligned) -
                               needed for correct deallocation */
}
IplImage136;


using namespace std;

ThreadedStaticBackgroundCompressor::ThreadedStaticBackgroundCompressor()  {
    background = NULL;
    bwbuffer = NULL;
    buffer1 = buffer2 = NULL;
    threshAboveBackground = threshBelowBackground = 0;
    smallDimMinSize = lgDimMinSize = 1;
    updateBackgroundFrameInterval = -1; // This means it never gets updated?.. Gets set to 1 by linearStackCompressor on start
    updateCount = 0;
    imOrigin.x = imOrigin.y = 0;
    processing_ = false;
    //Queue<processingInfo> piQueue ; //= new Queue<processingInfo>;
    //Queue<processingInfo> piQueue = new Queue<processingInfo> ;
//    std::thread ya(frameCompressionFunction, std::ref(piQueue));
//    auto L1 = std::bind(frameCompressionFunction, std::ref(piQueue));

}

//ThreadedStaticBackgroundCompressor::ThreadedStaticBackgroundCompressor(const ThreadedStaticBackgroundCompressor& orig) {
//}

ThreadedStaticBackgroundCompressor::~ThreadedStaticBackgroundCompressor() {

	// We are waiting for our workers to finish!
	for (int t=0; t< threadNumber ; t++) {
		if (threadArray[t].joinable()) {
			threadArray[t].join();
		}
	}


	cvReleaseImage(&background);
  //  cout << "size of imsToProcess is " << imsToProcess.size() << "\n";
    for (vector<InputImT>::iterator it = imsToProcess.begin(); it != imsToProcess.end(); ++it) {
        IplImage *im = it->first;
        cvReleaseImage(&im);
        if (it->second != NULL) {
            delete it->second;
        }
    }
  //  cout << "size of bri is " << bri.size() << "\n";
    for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
        delete (*it);
        *it = NULL;
    }

}

void ThreadedStaticBackgroundCompressor::calculateBackground() {
    if (imsToProcess.empty()) {
        return;
    }
    if (background != NULL) {
        cvReleaseImage(&background);
    }
 //   background = cvCloneImage(imsToProcess.front());
   for (vector<InputImT>::iterator it = imsToProcess.begin(); it != imsToProcess.end(); ++it) {
   //     cvMin(*it, background, background);
       updateBackground(it->first);
    }
}
void ThreadedStaticBackgroundCompressor::updateBackground(const IplImage* im) {
    if (background == NULL) {
        background = cvCloneImage(im);
    } else {
        cvMin(im, background, background);
    }
}


void ThreadedStaticBackgroundCompressor::addFrame(const IplImage* im, ImageMetaData *metadata) {
	IplImage *imcpy = cvCloneImage(im);
    addFrame(&imcpy, metadata);
}

void ThreadedStaticBackgroundCompressor::addFrame(IplImage** im, ImageMetaData* metadata) {

	std::lock_guard<std::mutex> lock(imsToProcessLock) ;
	std::lock_guard<std::mutex> lock2(backgroundLock) ; // locking background

	// We load our stack w/ imsToProcess
//	imsToProcess.insert(imsToProcess.begin(), InputImT(*im,metadata));
	imsToProcess.push_back( InputImT(*im,metadata));
//	insert(imsToProcess.begin(), InputImT(*im,metadata));


	// with updateBackgroundFrameInterval set to 1 (default, the background updates every time)
	// in principle the background should be calculated before we start processing but we will lock it further
	if (updateBackgroundFrameInterval > 0 && updateCount == 0) {
        updateBackground(*im);
        updateCount = updateBackgroundFrameInterval;
    }
    --updateCount;
    *im = NULL;


}

// This will be our worker threads
// We take a little performance hit by sleeping a bit, makes coding easier
void ThreadedStaticBackgroundCompressor::frameCompressionFunction() {

//	ROS_INFO("We start out thread!");
	processingInfo pi ; //= processingInfo();

	//ROS_WARN("We started our thread!");
	while (true) {

		piQueueLock.lock();
		bool newFood = !piQueue.empty();
		if (newFood) {
			//ROS_INFO("We get new food!!");
			pi = piQueue.front() ;
			piQueue.pop_front();
			piQueueLock.unlock();

		} else {
			piQueueLock.unlock();
			//ROS_INFO("We exit our thread!");
			return;
		}

		// we got stuff to do!
		BackgroundRemovedImage *brim = new BackgroundRemovedImage(pi.im, pi.sbc->background, pi.sbc->threshBelowBackground, pi.sbc->threshAboveBackground,
				pi.sbc->smallDimMinSize, pi.sbc->lgDimMinSize,  pi.metadata);

		briLock.lock();
		// We could just put it directly on the bri stack
		pi.sbc->bri[pi.imageNumber] = brim;
		briLock.unlock();
		cvReleaseImage(&(pi.im));

		//ROS_INFO("We processed our stack!");

		// We check here if we are done with work! Not the best design, but just may work
		//			if (framesWaitingToProcess() == 0 ) {
		//				return;
		//			}
	}
}

//void frameCompressionFunction(Queue<processingInfo> &sharedQueue, std::condition_variable & condVar) {
//
//	while(true) {
//		// This is the lock for the destination image?
////		std::unique_lock<std::mutex> lk(lockForCond);
//
//		// We now wait for notification
//		condVar.wait(lk);
//
//		ROS_WARN("We are here!");
//
////		auto ya = sharedQueue;
//		//std::this_thread::sleep_for(std::chrono::milliseconds(300));
//	}
//}

/*
 * In the non-threaded version, we do a frame by frame situation. In this threaded version we are going
 * to trigger our thread pool and send them all our images. This works because we only call this function
 * once we're past the keyframe, so effectively, how many images we have at this point is how many we
 * are going to process.
 * In this implementation, we put all the images on the stack we need to access and then tell all the workers to use them
 * we don't need to worry about putting new elements while we ask if there's stuff there
 */
// Trying approach without Queue, just protecting the imsToProcess container w/ mutexes
int ThreadedStaticBackgroundCompressor::processFrame() {

	//std::lock_guard<std::mutex> lock(processingLock);
	// if we are already processing we skip since we've done all we needed

	processingLock.lock();
	bool areWeProcessing = processing_;
	processingLock.unlock();
	if (areWeProcessing) {
		return numToProccess() ;
//		return imsToProcess.size();
	} else { // this should only run once

		processingLock.lock();
		processing_ = true;
		processingLock.unlock();

		// We start out worker threads here if we're just starting out

		// we will need to use mutexes to protect A LOT of this
		if (imsToProcess.empty()) {
			return 0;
		}
		if (background == NULL) {
			return -1;
		}

		// This makes sure we have space to put our processed image BACK where it belongs :)
		bri.resize(MAX(imsToProcess.size(), bri.size()), NULL);

		// We want to encapsulate it in the processingInfo so we can know which frame number, etc
		// We load up our queue until it's all full !
		piQueueLock.lock();
		while (!imsToProcess.empty()) {

			InputImT nextim = imsToProcess.back();
		//    IplImage *im = imsToProcess.back();
			imsToProcess.pop_back();
			IplImage *im = nextim.first;
			ImageMetaData *metadata = nextim.second;
			int imnum = imsToProcess.size();
			processingInfo *pi = new processingInfo(imnum, nextim.first, nextim.second, this);

			// This is my queue that my threaded-workers will consume, HAPPILY
			//piQueue.push_front(*pi);
			piQueue.push_back(*pi);

		}

		piQueueLock.unlock();
		processing_ = true; // We should set this before we launch our threaded workers

		if (!piQueue.empty()  )
		{
			totalFramesToProcess = imsToProcess.size() ;

			// We start our threads
			for (int t=0; t< threadNumber ; t++) {
				threadArray[t] = std::thread(&ThreadedStaticBackgroundCompressor::frameCompressionFunction, this);
			}

		}
		std::lock_guard<std::mutex> lock(piQueueLock);
		return piQueue.size(); // We return the size of the queue that we are processing now vs imsToProcess in single threaded version

	}
}

void ThreadedStaticBackgroundCompressor::changeBackground(const IplImage* newBackground) {
    IplImage *oldbak;
    oldbak = cvCloneImage(background);
    cvMin(newBackground, oldbak, background);
    BackgroundRemovedImage *bakim = new BackgroundRemovedImage(oldbak, background, threshBelowBackground, threshAboveBackground, smallDimMinSize, lgDimMinSize,  NULL);

    for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
      //  cout << "calling merge regions" << endl << flush;
        (*it)->mergeRegions(bakim, cvMax);       
       // cout << "merge regions returned" << endl << flush;
    }
    
    cvReleaseImage(&oldbak);
    delete bakim;
}

void ThreadedStaticBackgroundCompressor::mergeStacks(std::vector<ThreadedStaticBackgroundCompressor*> alreadyCompressedStacks) {

    Timer mergeTime = Timer();
    mergeTime.start();

	if (alreadyCompressedStacks.empty()) {
        return;
    }
    IplImage *newbak = cvCloneImage(background);
  //  cout << "calculating background" << endl<< flush;
    for (vector<ThreadedStaticBackgroundCompressor*>::iterator it = alreadyCompressedStacks.begin(); it != alreadyCompressedStacks.end(); ++it) {
        cvMin(newbak, (*it)->background, newbak);
    }
  //  cout << "changing backgrounds" << endl << flush;
    changeBackground(newbak);
    for (vector<ThreadedStaticBackgroundCompressor*>::iterator it = alreadyCompressedStacks.begin(); it != alreadyCompressedStacks.end(); ++it) {
        (*it)->changeBackground(newbak);
        bri.insert(bri.end(), (*it)->bri.begin(), (*it)->bri.end());
        (*it)->bri.clear();
    }
    cvReleaseImage(&newbak); 
    
    ostringstream os;
    os << mergeTime.getElapsedTimeInMilliSec() << " ms taken doing the mergeStacks";
    ROS_WARN(os.str().c_str());
    
}

void ThreadedStaticBackgroundCompressor::processFrames() {
    while (processFrame() > 0) {
        //process Frame does all the work
        ;
    }
}

void ThreadedStaticBackgroundCompressor::toDisk(std::ofstream& os) {

	// Sometimes we finish this before we have number of required numframes, so we must deal with this
	// We will wait for all of our workers to be done
	// We are waiting for our workers to finish!
	for (int t=0; t< threadNumber ; t++) {
		if (threadArray[t].joinable()) {
			threadArray[t].join();
		}
	}


	HeaderInfoT hi;
    hi.idcode = idCode();
    hi.numframes = bri.size();
    hi.headerSize = headerSizeInBytes;

    std::ofstream::pos_type start_loc = os.tellp();
    char zero[headerSizeInBytes] = {0};
    os.write(zero, headerSizeInBytes);
    cvResetImageROI(background);
    background->roi = NULL;
    writeIplImageToByteStream(os, background);
    for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
        (*it)->toDisk(os);
    }
    std::ofstream::pos_type end_loc = os.tellp();
    hi.totalSize = end_loc - start_loc;
    os.seekp(start_loc);
    os.write((char *) &hi, sizeof(hi));
    os.seekp(end_loc);
}

std::string ThreadedStaticBackgroundCompressor::saveDescription() {
  //  cout << "entered sbc save description\n";
    std::stringstream os;
    os << "Stack of common background images, beginning with this header:\n" << headerDescription();
 //   cout << "Stack of common background images, beginning with this header:\n" << headerDescription();
    os << "Then the background image, as an IplImage, starting with the " << sizeof (IplImage136) << " byte image header, followed by the image data\n";
//    cout << "Then the background image, as an IplImage, starting with the " << sizeof (IplImage) << " byte image header, followed by the image data\n";
    os << "Then nframes background removed images containing only differences from the background, in this format:\n";
//    cout << "Then nframes background removed images containing only differences from the background, in this format:\n";
    if (bri.empty()) {
        os << "<no background removed images in stack>\n";
    } else {
 //       cout << "bri.front = " << (int) bri.front();
        if (bri.front() == NULL) {
            os << "<background removed image is a NULL pointer>\n";
        } else {
            os << bri.front()->saveDescription();

        }
    }
  //  cout << "ended sbc save description\n";
    return os.str();
}
std::string ThreadedStaticBackgroundCompressor::headerDescription() {
    std::stringstream os;
    os << headerSizeInBytes << " byte zero-padded header, with the following fields (all " << sizeof(int) << " byte ints, except idcode):\n";
    os << sizeof(uint32_t) << " byte uint32_t idcode = " << hex << idCode() << dec << ", header size in bytes, total size of stack on disk, nframes: number of images in stack\n";
    return os.str();
}

ThreadedStaticBackgroundCompressor::HeaderInfoT ThreadedStaticBackgroundCompressor::getHeaderInfo(std::ifstream& is) {
    std::ifstream::pos_type start_loc = is.tellg();
    HeaderInfoT hi;
    is.read((char *) &hi, sizeof(hi));

    is.seekg(start_loc);
    return hi;
}

ThreadedStaticBackgroundCompressor * ThreadedStaticBackgroundCompressor::fromDisk(std::ifstream& is) {
    std::ifstream::pos_type start_loc = is.tellg();
    HeaderInfoT hi;

    hi = getHeaderInfo(is);
   // cout << "header size is " << hi.headerSize << "  id code is " << hex << hi.idcode <<dec<< "  numframes = " << hi.numframes << std::endl;
    is.seekg(start_loc + (std::ifstream::pos_type) hi.headerSize);
    ThreadedStaticBackgroundCompressor *sbc = new ThreadedStaticBackgroundCompressor();
 //   cout << "reading background" << endl;
    sbc->background = readIplImageFromByteStream(is);
//    cout << "background read in" << endl;
    for (int j = 0; j < hi.numframes; ++j) {
        //BackgroundRemovedImage *bri = BackgroundRemovedImage::fromDisk(is, sbc->background);
        BackgroundRemovedImage *bri = BackgroundRemovedImageLoader::fromFile(is, sbc->background);
        sbc->bri.push_back(bri);
    }
    return sbc;
}
//estimate, does not include metadata and maybe some other stuff
int ThreadedStaticBackgroundCompressor::sizeInMemory() {
    std::lock_guard<std::mutex> lock(briLock);

    int totalBytes = sizeof(this);
    if (background != NULL) {
        totalBytes += (sizeof(IplImage) + background->imageSize) * (imsToProcess.size()+4);
    }
//    mutexBackgroundRemovedImageStack.lock();
     for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
        if ((*it) != NULL) {
    	 totalBytes += (*it)->sizeInMemory();

        }
     }
//     mutexBackgroundRemovedImageStack.unlock();
    return totalBytes;
}

int ThreadedStaticBackgroundCompressor::sizeOnDisk() {
    
   int totalBytes = headerSizeInBytes + ((background != NULL) ? sizeof(IplImage) + background->imageSize : 0);
   for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
        totalBytes += (*it)->sizeOnDisk();
    }
    return totalBytes;
}

void ThreadedStaticBackgroundCompressor::writeIplImageToByteStream(std::ofstream& os, const IplImage *src) {
   assert(src != NULL);
   ofstream::pos_type cloc = os.tellp(); 
   // We first convert this to a packed header so that we get 136
   // This should have been serialized to avoid these pitfalls
   IplImage136 imout;
   imout.ID = src->ID;
   imout.align = src->align;
   imout.alphaChannel = src->alphaChannel;
   imout.dataOrder = src->dataOrder;
   imout.depth = src->depth;
   imout.height = src->height;
   imout.imageSize = src->imageSize;
   imout.nChannels = src->nChannels;
   imout.nSize = sizeof(imout);
   imout.origin = src->origin;
   imout.width = src->width;
   imout.widthStep = src->widthStep;
   for (int j = 0; j < 4; ++j) {
       imout.BorderConst[j] = src->BorderConst[j];
       imout.BorderMode[j] = src->BorderMode[j];
       imout.channelSeq[j] = src->channelSeq[j];
       imout.colorModel[j] = src->colorModel[j];
   }
   imout.imageData = NULL;
   imout.imageDataOrigin = NULL;
   imout.imageId = NULL;
   imout.maskROI = NULL;
   imout.roi = NULL;
   imout.tileInfo = NULL;

   // We write our header
   os.write((char *) &imout, sizeof(IplImage136));
   // we can write using the original iplImage
   os.write((char *) src->imageData, src->imageSize);

//   os.write((char *) src, sizeof(IplImage));
//   os.write((char *) src->imageData, src->imageSize);

 }

IplImage * ThreadedStaticBackgroundCompressor::readIplImageFromByteStream(std::ifstream& is) {
    return IplImageLoaderFixedWidth::loadIplImageFromByteStream(is);
}

void ThreadedStaticBackgroundCompressor::playMovie(const char* windowName) {
    if (windowName == NULL) {
        windowName = "Movie of stack";
    }
    cvNamedWindow(windowName, 0);
    IplImage *im = NULL;
     for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
         (*it)->restoreImage(&im);
         cvShowImage(windowName, im);
         cvWaitKey(50);
     }
}

// changed this since it may be not yet entered
int ThreadedStaticBackgroundCompressor::numProcessed() {
	std::lock_guard<std::mutex> lock(briLock) ;

	int numFramesProcessed = 0 ;
	for (vector<BackgroundRemovedImage *>::iterator it = bri.begin(); it != bri.end(); ++it) {
	         if ( !(*it) == NULL) {
	        	 numFramesProcessed++ ;
	         }
	}
//	return bri.size();
	return numFramesProcessed ;
}

// Number of frames ready to process
// This is now a tricky call because we decoupled them
int ThreadedStaticBackgroundCompressor::numToProccess() {
	std::lock_guard<std::mutex> lock(piQueueLock);
	std::lock_guard<std::mutex> lock_two(processingLock);

	if (!processing_) {
		return imsToProcess.size();
	} else {
		return piQueue.size();
//		return totalFramesToProcess - numProcessed() ;
	}
}

bool ThreadedStaticBackgroundCompressor::framesWaitingToProcess() {
	std::lock_guard<std::mutex> lock(piQueueLock);
	return piQueue.size();

}

void ThreadedStaticBackgroundCompressor::reconstructFrame(int frameNum, IplImage** dst) {
    if (frameNum < 0 || frameNum >= bri.size()) {
        if (*dst != NULL) {
            cvReleaseImage(dst);
        }
        *dst = NULL;
        return;
    }
    BackgroundRemovedImage *brim = bri.at(frameNum);
    brim->restoreImage(dst);
}

/*
const IplImage *StaticBackgroundCompressor::getBackground() {
    return this->background;
}
 * */
void ThreadedStaticBackgroundCompressor::copyBackground(IplImage** dst) {
    if (dst == NULL) {
        return;
    }
    if (background == NULL) {
        if (*dst != NULL) {
            cvReleaseImage(dst);
            *dst = NULL;
        }
        return;
    }
    setImageOriginFromBRI();
    if (*dst == NULL || (*dst)->width != background->width + imOrigin.x || (*dst)->height != background->height + imOrigin.y || (*dst)->depth != background->depth || (*dst)->nChannels != background->nChannels) {
        if (*dst != NULL) {
            cvReleaseImage(dst);
        }
        *dst = cvCreateImage(cvSize(background->width + imOrigin.x, background->height+imOrigin.y), background->depth, background->nChannels);
    }
    cvSetZero(*dst);
    CvRect r; r.x = imOrigin.x; r.y = imOrigin.y; r.width = background->width; r.height = background->height;
    CvRect roi = cvGetImageROI(*dst);
    cvSetImageROI(*dst, r);
    cvCopyImage(background, *dst);
    cvSetImageROI(*dst, roi);
}

void ThreadedStaticBackgroundCompressor::annotatedFrame(int frameNum, IplImage** buffer, IplImage** annotatedImage) {
    reconstructFrame(frameNum, buffer);
    if (*buffer == NULL) {
        if (*annotatedImage != NULL) {
            cvReleaseImage(annotatedImage);        
            *annotatedImage = NULL;
            return;
        }
    }
    if (*annotatedImage == NULL || (*annotatedImage)->width != (*buffer)->width || (*annotatedImage)->height != (*buffer)->height) {
        if (*annotatedImage != NULL) {
            cvReleaseImage(annotatedImage);
        }
        *annotatedImage = cvCreateImage(cvGetSize(*buffer), (*buffer)->depth, 3);
    }
    cvConvertImage(*buffer, *annotatedImage,0);

    BackgroundRemovedImage *brim = bri.at(frameNum);
    brim->annotateImage(*annotatedImage);
}

const ImageMetaData *ThreadedStaticBackgroundCompressor::getMetaData(int frameNumber) {
    BackgroundRemovedImage *brim = bri.at(frameNumber);
    if (brim == NULL) {
        return NULL;
    }
    return brim->getMetaData();
}

int ThreadedStaticBackgroundCompressor::numRegionsInFrame(int frameNum) const {
    if (frameNum < 0 || frameNum >= bri.size()) {
        return -1;
    }
    const BackgroundRemovedImage *brim = bri.at(frameNum);
    return brim->numRegions();
}

void ThreadedStaticBackgroundCompressor::setImageOriginFromBRI() {
    if (bri.empty()) {
        return;
    }
    imOrigin = bri.front()->getImageOrigin();
}

CvSize ThreadedStaticBackgroundCompressor::getFrameSize() {
    setImageOriginFromBRI();
    if (background == NULL) {
        return cvSize(0,0);
    }
    CvSize sz = cvGetSize(background);
   // cout << endl << "sz.width = " << sz.width << ", sz.height = " << sz.height << endl;
    IplImage *im = background;
   // cout << "background params: w= " << im->width << ", h= " << im->height << ", nchannels = " << im->nChannels << ", width step = " << im->widthStep << "imageSize = " << im->imageSize << endl;

    sz.width += imOrigin.x;
    sz.height += imOrigin.y;
    //cout << endl << "imorigin.x= " << imOrigin.x << ", imorigin.y= " << imOrigin.y << ", sz.width = " << sz.width << ", sz.height = " << sz.height << endl;
    return sz;
}

CvRect ThreadedStaticBackgroundCompressor::getValidRoi() {
    CvRect r;
    r.x = r.y = r.width = r.height = 0;
    if (background == NULL) {
        return r;
    }
    setImageOriginFromBRI();
    CvSize sz = cvGetSize(background);
    r.x = imOrigin.x; r.y = imOrigin.y; r.width = sz.width; r.height = sz.height;
    return r;
}
