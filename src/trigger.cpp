#include "capture.h"

void Capture::initiateTriggering() {
    if (triggeringState == Inactive) {
        trig_index = 0;

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        
	int s = pthread_create(&triggeringThread, &attr, &Capture::getTriggerHelper, this);
        if (s != 0) {
            printf("Error %d: Could not create a thread for triggering\n", s);
        }

        pthread_attr_destroy(&attr);
    }
}

void* Capture::getTriggerFrame() {
    PXL_RETURN_CODE rc;
    U32 previousFrame = 0;
    triggeringState = InProgress;

    while(triggeringState != Inactive){

        U32 rc = PxLGetNextFrame(camera->hCamera,
                             camera->rawImageSize,
                             &camera->frameBuffer[0],
                             &camera->frameDesc);
        triggerBuffer[trig_index] = camera->frameBuffer;
        trigFrameNum[trig_index] = camera->frameDesc.uFrameNumber;

        if (camera->frameDesc.uFrameNumber - previousFrame > 1) {
            camera->trigMisses++;
            printf("Camera Dropped Frame\n");
        }
	else
            printf("Camera got image %u\n", camera->frameDesc.uFrameNumber);
                
        previousFrame = camera->frameDesc.uFrameNumber;

	sem_post(&proc_full);
            
        trig_index = (trig_index + 1) % TRIG_BUFF_SIZE;
    }

    triggeringState = Inactive;
    return nullptr;
}
