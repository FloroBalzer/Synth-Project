# Report

## Table of Contents <!-- omit from toc -->
- [Tasks](#tasks)
  - [`SampleISR`](#sampleisr)
  - [`CAN_RX_ISR`](#can_rx_isr)
  - [`CAN_TX_ISR`](#can_tx_isr)
  - [`CAN_TX_Task`](#can_tx_task)
  - [`scanKeysTask`](#scankeystask)
  - [`displayUpdateTask`](#displayupdatetask)
  - [`decodeTask`](#decodetask)
  - [A characterisation of each task with its theoretical minimum initiation interval and measuredmaximum execution time (this should go in the tasks section above)](#a-characterisation-of-each-task-with-its-theoretical-minimum-initiation-interval-and-measuredmaximum-execution-time-this-should-go-in-the-tasks-section-above)
- [A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are metunder worst-case conditions](#a-critical-instant-analysis-of-the-rate-monotonic-scheduler-showing-that-all-deadlines-are-metunder-worst-case-conditions)
- [A quantification of total CPU utilisation](#a-quantification-of-total-cpu-utilisation)
- [Shared Data Structures](#shared-data-structures)
- [An analysis of inter-task blocking dependencies that shows any possibility of deadlock](#an-analysis-of-inter-task-blocking-dependencies-that-shows-any-possibility-of-deadlock)

## Tasks
___

### `SampleISR`

This task is responsible for generating the voltage written to the speaker. It is important for this to be very fast and regular to produce the desired audio waves therefore, it is implemented as an interrupt task at a rate of 22 KHz.

### `CAN_RX_ISR`

This is a small task that interrupts every time there is a received message on the CAN bus. It simply adds it to an incoming queue 

### `CAN_TX_ISR`

This interrupt task is called every time there is an outgoing tranmission on the CAN bus. 

### `CAN_TX_Task`

This task is run on a thread and runs every time a transmission is added to the outgoing queue.

### `scanKeysTask`

This task is threadded and is used to scan the board for all the inputs and resolve them. It needs to be fast enough to accurately detect key presses and releases, it runs every 20 ms.

### `displayUpdateTask`

The `displayUpdateTask` controls the display of the board and runs on it's own thread every 100 ms.

### `decodeTask`

This task is also threaded and runs everytime there is a mesage received into the incoming queue.

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


