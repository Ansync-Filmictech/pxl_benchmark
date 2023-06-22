#include <iostream>
#include "camera.h"
#include "capture.h"
#include "sys/time.h"

#define UNUSED(x) (void)(x)
long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    // printf("milliseconds: %lld\n", milliseconds);
    return milliseconds;
}

int main(int argc, char* argv[]) {
    auto camera = new Camera();
    auto capture = new Capture(camera);

    //Initialize Objects
    camera->initialize();
    camera->setTriggering();

    //Start Threads
    capture->startInspection();
    
    //Run for 60s
    long long timestamp = current_timestamp();
    while((current_timestamp() - timestamp) < 60000){}
   
    //Stop threads 
    capture->initiateStopInspection();
    U32 rc = PxLUninitialize(camera->hCamera);

    printf("Lost %d frames over 60s\n", camera->trigMisses);
    return 0;
}

