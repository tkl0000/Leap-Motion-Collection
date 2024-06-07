/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "LeapC.h"
#include "ExampleConnection.h"

#include <math.h>
#include <time.h>

int64_t lastFrameID = 0; //The last frame received

LEAP_QUATERNION multiply(LEAP_QUATERNION *a, LEAP_QUATERNION *b) {
    LEAP_QUATERNION lq;
    lq.w = (a->w)*(b->w) - (a->x)*(b->x) - (a->y)*(b->y) - (a->z)*(b->z);
    lq.x = (a->x)*(b->w) + (a->w)*(b->x) + (a->y)*(b->z) - (a->z)*(b->y);
    lq.y = (a->w)*(b->y) - (a->x)*(b->z) + (a->y)*(b->w) + (a->z)*(b->x);
    lq.z = (a->w)*(b->z) + (a->x)*(b->y) - (a->y)*(b->x) + (a->z)*(b->w);
    return lq;
}

LEAP_QUATERNION *conjugate(LEAP_QUATERNION *lq) { //changes quaternion to conjugate of itself (inverses imaginary values)
    lq->x *= -1;
    lq->y *= -1;
    lq->z *= -1;
    return lq;
}

float flexion(LEAP_BONE *a, LEAP_BONE *b) { //returns angle between two bones in radians
    LEAP_QUATERNION *q_A = &a->rotation;
    LEAP_QUATERNION *q_B = &b->rotation;
    LEAP_QUATERNION lq = multiply(conjugate(q_A), q_B);
    float ret = 2*acos(lq.w);
    return ret;
}

float calcAngle(LEAP_VECTOR *a, LEAP_VECTOR *b) { //give angle between two vectors
  float x1 = a->x, y1 = a->y, z1 = a->z;
  float x2 = b->x, y2 = b->y, z2 = b->z;
  return acos((x1*x2 + y1*y2 + z1*z2)/(sqrt(pow(x1, 2)+pow(y1, 2)+pow(z1, 2)) * sqrt(pow(x2, 2)+pow(y2, 2)+pow(z2, 2))));
}

double PCFreq = 0.0;
__int64 CounterStart = 0;

void StartCounter()
{
    LARGE_INTEGER li;
    if(!QueryPerformanceFrequency(&li))
    printf("QueryPerformanceFrequency failed!\n");

    PCFreq = (double)(li.QuadPart);

    QueryPerformanceCounter(&li);
    CounterStart = li.QuadPart;
}
double GetCounter()
{
    LARGE_INTEGER li;
    QueryPerformanceCounter(&li);
    return (double)(li.QuadPart-CounterStart)/PCFreq;
}

int main(int argc, char** argv) {
  OpenConnection(eLeapTrackingMode_Desktop);
  while(!IsConnected)
    millisleep(100); //wait a bit to let the connection complete

  printf("Connected.");
  LEAP_DEVICE_INFO* deviceProps = GetDeviceProperties();
  if(deviceProps) {
    printf("Using device %s.\n", deviceProps->serial);
  }

  char path[128];
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  float timeout;
  if (argc == 3) {
    sprintf(path, argv[1]);
    timeout = atof(argv[2]);
  }
  else return 1;

  FILE *recordFilePointer;
  recordFilePointer = fopen(path, "w+");
  const char header[1024] = 
  "time,palm_position_x,palm_position_y,palm_position_z,palm_normal_x,palm_normal_y,palm_normal_z,palm_direction_x,palm_direction_y,palm_direction_z,thumb_extension,index_extension,middle_extension,ring_extension,pinky_extension";
  fprintf(recordFilePointer, header);
  fprintf(recordFilePointer, "\n");

  StartCounter();

  for(;;){
    LEAP_TRACKING_EVENT *frame = GetFrame();
    if(frame && (frame->tracking_frame_id > lastFrameID)){
      lastFrameID = frame->tracking_frame_id;
      if (frame->nHands == 1) {
        LEAP_HAND *hand = frame->pHands;
        LEAP_PALM *palm = &hand->palm;
        fprintf(recordFilePointer, "%f,", GetCounter); // time
        fprintf(recordFilePointer, "%f,%f,%f,", palm->position.x, palm->position.y, palm->position.z); // palm position
        fprintf(recordFilePointer, "%f,%f,%f,", palm->normal.x, palm->normal.y, palm->normal.z); // palm normal
        fprintf(recordFilePointer, "%f,%f,%f,", palm->direction.x, palm->direction.y, palm->direction.z); // palm direction
        fprintf(recordFilePointer, "%f,", hand->grab_angle); // grab angle
        fprintf(recordFilePointer, "%f,", hand->grab_strength); // grab strength
        fprintf(recordFilePointer, "%f,", hand->pinch_distance); // pinch distance
        fprintf(recordFilePointer, "%f,", hand->pinch_strength); // pinch strength
        fprintf(recordFilePointer, "%d,", hand->thumb.is_extended); // thumb extension
        fprintf(recordFilePointer, "%d,", hand->index.is_extended); // index extension
        fprintf(recordFilePointer, "%d,", hand->middle.is_extended); // middle extension
        fprintf(recordFilePointer, "%d,", hand->ring.is_extended); // ring extension
        fprintf(recordFilePointer, "%d,", hand->pinky.is_extended); // pinky extension
        fprintf(recordFilePointer, "\n"); // new line
      }
    }

    if (((double)(clock()))/CLOCKS_PER_SEC > timeout) { // timeout
      fflush(recordFilePointer);
      fclose(recordFilePointer);
      return 0;
    }

  } //ctrl-c to exit
  fclose(recordFilePointer);
  return 0;
}
//End-of-Sample
