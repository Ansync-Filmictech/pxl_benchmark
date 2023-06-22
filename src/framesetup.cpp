#include "capture.h"
#include <fcntl.h>

// CAPTURE DEFINITIONS FOR FRAME PROCESS
#include <sys/time.h>

void Capture::initiateFrameProcess() {
    if (procFrameState == Inactive) {
        sem_init(&proc_empty, 0, TRIG_BUFF_SIZE);
        sem_init(&proc_full, 0, 0);
        proc_index = 0;

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        int s = pthread_create(&procFrameThread, &attr, &Capture::procFrameHelper, this);
        if (s != 0) {
            printf("Error %d: Could not create a thread for processing\n", s);
       }

        pthread_attr_destroy(&attr);
    }
}


//This function converts, encodes, and store the image matrix
//To improve speed we can and have seperated them into individual threads
//For the purpose of this test they are combine in 1 location for simplicity
void* Capture::procFrame() {
    procFrameState = InProgress;
    
    while (procFrameState != Inactive ) {
        sem_wait(&proc_full);
        if(triggeringState == Inactive) {
            break;
        }

	//Convert image to RGB
	auto img1 = new cv::Mat(matSize, CV_8UC1, triggerBuffer[proc_index].data());
	cv::cvtColor(*img1, procBuffer[proc_index], cv::COLOR_BayerRG2RGB);

	//Generate file name 
	std::ostringstream frameNumSS;
        frameNumSS << std::internal << std::setfill('0') << std::setw(5) << trigFrameNum[proc_index];
        std::string fileName;
       	fileName = frameNumSS.str() + ".bmp";

	//Encode
	cv::imencode(".bmp", procBuffer[proc_index], writerBuffer[proc_index]);

	char* buff;
	int buff_size;	
	buff = reinterpret_cast<char *>(&writerBuffer[write_index][0]);
        buff_size = writerBuffer[write_index].size();

	//Write
	int wfd = open(fileName.c_str(), O_WRONLY|O_CREAT|O_ASYNC, S_IRGRP|S_IWGRP|S_IRUSR|S_IWUSR);
        if(wfd < 0){
		printf("file open error\n");
		continue;
	}
	
	int ret = write(wfd, buff, buff_size);
	if(ret < 0){
		printf("write error\n");
		continue;
	}
        close(wfd);
	
	img1->release();
	
	proc_index = (proc_index + 1) % TRIG_BUFF_SIZE;
    }

    procFrameState = Inactive;
    return nullptr;
}
