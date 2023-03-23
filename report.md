# Report

## Table of Contents <!-- omit from toc -->
- [Tasks](#tasks)
  - [A characterisation of each task with its theoretical minimum initiation interval and measuredmaximum execution time (this should go in the tasks section above)](#a-characterisation-of-each-task-with-its-theoretical-minimum-initiation-interval-and-measuredmaximum-execution-time-this-should-go-in-the-tasks-section-above)
- [A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are metunder worst-case conditions](#a-critical-instant-analysis-of-the-rate-monotonic-scheduler-showing-that-all-deadlines-are-metunder-worst-case-conditions)
- [A quantification of total CPU utilisation](#a-quantification-of-total-cpu-utilisation)
- [Shared Data Structures](#shared-data-structures)
- [An analysis of inter-task blocking dependencies that shows any possibility of deadlock](#an-analysis-of-inter-task-blocking-dependencies-that-shows-any-possibility-of-deadlock)

## Tasks
___

| Name              | Type of Task | Rate of execution                             | Minimum Initiation Interval  | Maximum Execution Time (ms) |
|-------------------|--------------|-----------------------------------------------|------------------------------|-----------------------------|
| SampleISR         | Interrupt    | 22 KHz                                        |                              |                             |
| CAN_RX_ISR        | Interrupt    | After a message is received on the can bus    |                              |                             |
| CAN_TX_ISR        | Interrupt    | After a message is transmitted on the CAN bus |                              |                             |
| CAN_TX_Task       | Thread       | When a message is added to the outgoing queue |                              |                             |
| scanKeysTask      | Thread       | 20ms                                          |                              | 12.725                      |
| displayUpdateTask | Thread       | 100ms                                         |                              |                             |
| decodeTask        | Thread       | When a message is added to the incoming queue |                              |                             |

### A characterisation of each task with its theoretical minimum initiation interval and measuredmaximum execution time (this should go in the tasks section above)



## A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are metunder worst-case conditions
___

## A quantification of total CPU utilisation
___


## Shared Data Structures
___

| Type     | Name                      | Protected by       |
|----------|---------------------------|--------------------|
| uint8_t  | keyArray[7]               | keyArrayMutex      |
| uint32_t | currentStepSize[key_size] | Atomic stores only |
| uint8_t  | pressed[key_size]         | pressedMutex       |
| uint8_t  | position                  | Atomic stores only |
| bool     | receiver                  | Atomic stores only |
| bool     | position_set              | Atomic stores only |
| bool     | west                      | Atomic stores only |
| bool     | east                      | Atomic stores only |
| uint8_t  | RX_Mesage[8]              | keyArrayMutex      |
| int      | bend                      | Atomic stores only |
| int      | joyXbias                  | Atomic stores only |

In addition to this each knob defined as type Knob, where the Knob class is made safe by using atomic stores only.

## An analysis of inter-task blocking dependencies that shows any possibility of deadlock
___

