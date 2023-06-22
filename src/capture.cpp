#include "capture.h"

Capture::Capture(Camera *_camera) :
    camera(_camera)
    {}

void Capture::initialize() {
    matSize = cv::Size(static_cast<int>(camera->rawImageWidth), static_cast<int>(camera->rawImageHeight));

    for (int i = 0; i < TRIG_BUFF_SIZE; i++) {
        triggerBuffer[i].resize(camera->rawImageSize);
        procBuffer[i].resize(camera->rawImageSize*3);
    }
}

bool Capture::startInspection() {
    initialize();
    initiateTriggering();    //Image Capture
    initiateFrameProcess();  //Image Formatting/Writting
    return true;
}

void Capture::initiateStopInspection() {
    if (stopTriggerState == Inactive) {
        pthread_attr_t attr;

        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        int s = pthread_create(&stopTriggerThread, &attr, &Capture::stopInspectionHelper, this);
        if (s != 0) {
            printf("Error %d: Could not create a thread for triggering\n", s);
        }

        pthread_attr_destroy(&attr);
    }
}

void* Capture::stopInspection() {
    stopTriggerState = InProgress;
    camera->stopTriggering();
    
    triggeringState = Inactive;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto start = std::chrono::high_resolution_clock::now();

    while(procFrameState != Inactive) {
        sem_post(&proc_full);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
        if(duration.count() > 5) {
            pthread_cancel(procFrameThread);
            procFrameState = Inactive;
        }
    }

    for (int i = 0; i < TRIG_BUFF_SIZE; i++) {
        triggerBuffer[i].clear();

        procBuffer[i].release();

    }

    Proc = TRIG_BUFF_SIZE;

    sem_destroy(&proc_full);
    sem_destroy(&proc_empty);

    stopTriggerState = Inactive;
    return nullptr;
}

OneTimeState Capture::getTriggeringState() const {
    return triggeringState;
}

OneTimeState Capture::getProcFrameState() const {
    return procFrameState;
}
