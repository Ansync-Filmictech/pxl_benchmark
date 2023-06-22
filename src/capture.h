#ifndef CAPTURE_H
#define CAPTURE_H

#include <cstdio>
#include <stdexcept>
#include <unistd.h>
#include <cassert>
#include <vector>
#include <pthread.h>
#include <csignal>
#include <thread>
#include <semaphore.h>
#include <utility>

#include <sys/syscall.h>
#include <sys/types.h>
#include <ctime>
#include <sys/resource.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/ocl.hpp>
#include <PixeLINKApi.h>
#include <PixeLINKCodes.h>

#include "camera.h"

// Useful defines and enums.
#define API_RANGE_ERROR(rc) (((PXL_RETURN_CODE)(rc) == ApiInvalidParameterError) || ((PXL_RETURN_CODE)(rc) == ApiOutOfRangeError))
#define TRIG_BUFF_SIZE  12

class Capture {
  private:
    Camera* camera;

    cv::Size matSize;

    std::array<std::vector<U8>, TRIG_BUFF_SIZE> triggerBuffer;
    std::array<U32, TRIG_BUFF_SIZE> trigFrameNum{};

    std::array<cv::Mat, TRIG_BUFF_SIZE> procBuffer;
    std::array<U32, TRIG_BUFF_SIZE> procFrameNum{};
    
    std::array<cv::Mat, TRIG_BUFF_SIZE> encoderBuffer;
    std::array<U32, TRIG_BUFF_SIZE> encoderFrameNum{};

    std::array<cv::Mat, TRIG_BUFF_SIZE> detectBuffer;
    std::array<U32, TRIG_BUFF_SIZE> detectFrameNum{};
    
    std::array<std::vector<U8>, TRIG_BUFF_SIZE> writerBuffer;
    std::array<std::string, TRIG_BUFF_SIZE> writerFileNames{};

    std::array<cv::Mat, TRIG_BUFF_SIZE> previewBuffer;

    std::array<std::vector<float>, TRIG_BUFF_SIZE> audioWriteBuffer;

    size_t trig_index{},
           proc_index{},
	   encoder_index{},
	   preview_index{},
	   detect_index{},
	   aud_read_index{},
	   aud_write_index{},
	   param_index{},
	   write_index{};

    sem_t trig_set{};
    sem_t proc_full{}, proc_empty{};
    sem_t encoder_full{}, encoder_empty{};
    sem_t preview_full{}, preview_empty{};
    sem_t detect_full{}, detect_empty{};
    sem_t aud_read_full{}, aud_read_empty{};
    sem_t aud_write_full{}, aud_write_empty{};
    sem_t param_full{}, param_empty{};
    sem_t writer_full{}, writer_empty{};

    OneTimeState triggeringState{Inactive};
    pthread_t triggeringThread{};

    OneTimeState triggerSetterState{Inactive};
    pthread_t triggerSetterThread{};

    OneTimeState procFrameState{Inactive};
    pthread_t procFrameThread{};
    
    OneTimeState framePreviewState{Inactive};
    pthread_t framePreviewThread{};

    OneTimeState frameEncoderState{Inactive};
    pthread_t frameEncoderThread{};
    
    OneTimeState frameDetectState{Inactive};
    pthread_t frameDetectThread{};
    
    OneTimeState audioReadState{Inactive};
    pthread_t audioReadThread{};
    
    OneTimeState audioWriteState{Inactive};
    pthread_t audioWriteThread{};
    
    OneTimeState writeParamState{Inactive};
    pthread_t writeParamThread{};
    
    OneTimeState frameWriterState{Inactive};
    pthread_t frameWriterThread{};

    OneTimeState stopTriggerState{Inactive};
    pthread_t stopTriggerThread{};

    void* getTriggerFrame();
    static void* getTriggerHelper(void* nothing) {
        return ((Capture*)nothing)->getTriggerFrame();
    }

    void* triggerSetter();
    static void * triggerSetterHelper(void* nothing) {
        return ((Capture*)nothing)->triggerSetter();
    }

    void* procFrame();
    static void* procFrameHelper(void* nothing) {
        return ((Capture*)nothing)->procFrame();
    }

    void* stopInspection();
    static void* stopInspectionHelper(void* nothing) {
        return ((Capture*)nothing)->stopInspection();
    }

    void* frameEncoder();
    static void* frameEncoderHelper(void* ctx) {
	return ((Capture*)ctx)->frameEncoder();
    }
   
    void* framePreview();
    static void* framePreviewHelper(void* ctx) {
	return ((Capture*)ctx)->framePreview();
    }

    void* frameDetect();
    static void* frameDetectHelper(void* ctx) {
	return ((Capture*)ctx)->frameDetect();
    }

    void* audioRead();
    static void* audioReadHelper(void* ctx) {
	return ((Capture*)ctx)->audioRead();
    }

    void* writeParams();
    static void* writeParamsHelper(void* ctx) {
	return ((Capture*)ctx)->audioRead();
    }
    
    void* frameWriter();
    static void* frameWriterHelper(void* ctx) {
	return ((Capture*)ctx)->frameWriter();
    }    void initialize();
  
  public:
    explicit Capture(Camera *_camera);

    int Proc{TRIG_BUFF_SIZE},
	Encoder{TRIG_BUFF_SIZE},
	Preview{TRIG_BUFF_SIZE},
	Detect{TRIG_BUFF_SIZE},
	Aud_Read{TRIG_BUFF_SIZE},
	Aud_Write{TRIG_BUFF_SIZE},
	Params{TRIG_BUFF_SIZE},
	Writer{TRIG_BUFF_SIZE};

    bool startInspection();

    //Threaded Functions
    void initiateStopInspection();
    void initiateTriggering();
    void initiateTriggerSetter();
    void initiateFrameProcess();
    void initiateFrameEncoder();
    void initiateFramePreview();
    void initiateFrameDetect();
    void initiateAudioRead();
    void initiateAudioWrite();
    void initiateParamsProcess();
    void initiateFrameWriter();

    //GETTERS
    OneTimeState getTriggeringState() const;
    OneTimeState getProcFrameState() const;
    OneTimeState getFrameEncoderState() const;
    OneTimeState getFramePreviewState() const;
    OneTimeState getFrameDetectState() const;
    OneTimeState getAudioReadState() const;
    OneTimeState getAudioWriteState() const;
    OneTimeState getParamsState() const;
    
    enum class CaptureFilmSize {
        MM16 = 0,
        MM8 = 1,
        SUPER8 = 2
    }; 
  
    //Variable for Frame Detection 
    int perfPos{0}; 
    CaptureFilmSize filmSize{CaptureFilmSize::MM16};

    //Variable for Audio Read
    int prevL{0}, prevR{0}, yPos{0}, prevDiff{0};

    //Process Params Functions
    bool make_csv_file() const;
    bool make_crop_file() const;
    void write_error_to_csv(const char *errorMessage, const std::string &created, const std::string &id) const;
    void write_crop() const ;
    void write_csv() const ;

    //Process Params Variables
    float frameDrift{0.0f};
    std::string crop_string{""};
    std::string csv_string{""};
    std::ostringstream crop_ss;
    std::ostringstream csv_ss;
    std::string reelID;
    std::string serialID;
    std::string lgn;
    std::string copyright;
    std::string creator;
    std::string emulationPath;
 };

#endif // CAPTURE_H
