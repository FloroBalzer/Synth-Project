# Report

## Table of Contents <!-- omit from toc -->
- [Introduction:](#introduction)
- [Tasks](#tasks)
- [A critical instant analysis of the rate monotonic scheduler, showing that all deadlines are met under worst-case conditions](#a-critical-instant-analysis-of-the-rate-monotonic-scheduler-showing-that-all-deadlines-are-met-under-worst-case-conditions)
- [A quantification of total CPU utilisation](#a-quantification-of-total-cpu-utilisation)
- [Shared Data Structures](#shared-data-structures)
- [An analysis of inter-task blocking dependencies that shows any possibility of deadlock](#an-analysis-of-inter-task-blocking-dependencies-that-shows-any-possibility-of-deadlock)

## Introduction:

This implementation of a modular keyboard on the STM32 contains a variety of functions. The use of these functions will be explained in this section.

**Knobs:**

Counting from the left, 
- *Knob 1* : Volume Control: Set the output volume of the synth, varies between 0 and 8.
- *Knob 2:* Octave Control:* Selects the octave being played, from 1 to 7, with octave 4  having A at 440Hz.
- *Knob 3:* Waveform Select*. The design comes with 4 different waveforms with the following indexes: 0: Sawtooth, 1:Square, 2: Triangle, 3: Sine.
- *Knob 4:* Unassigned

**Joystick**

The input on the x-axis of the joystick gives a variable pitch bend to the tone being played. Pushing the joystick to the right will increase the pitch, while pushing it to the left will decrease it.

**Polyphony**

The synth is able to play multiple keys at once.

**Display**

The display shows the current values set on the knobs as V: Volume, O: Octave, W: Waveform. Additionally, it displays the current keys being played.

**Modular Option**

By connecting multiple keyboards together, the modules will autmatically detect each other and combine to give a full keyboard range, keeping all the above functions. The leftmost board becomes the master board, allowing its knobs to control every function across the whole range. The octave values are shifted accordingly to keep the pitches of each board in ascending order. It is recommended to reset all the boards at the same time once they are connected. When the board are used in a modular implementation, only the rightmost board will have an active display, showing the current setting and whether it is acting as a receiver or a sender. However, it will only be able to show the keys being played on itself.




## Timing and CPU Analysis
___

This table outlines the thread and ISR minimum initation and maximum execution times and uses these values to calculate the CPU Utilisation. As seen from the table the most demanding task is SampleISR as it has to operate at a much higher frequency. DisplayUpdateTask also has a conisderable contribution to the demand of the CPU as it has to generate every pixel. All the other tasks have a negligible demand on the CPU.  


| Name              | Type of Task | Rate of execution                             | Minimum Initiation Interval (ms) | Maximum Execution Time (ms) | CPU Utilisation (%) |
|-------------------|--------------|-----------------------------------------------|----------------------------------|-----------------------------|---------------------|
| SampleISR         | Interrupt    | 22 KHz                                        | 0.045                            | 0.021                       | 46.67               |
| CAN_RX_ISR        | Interrupt    | After a message is received on the can bus    | 0.512                            | 0.010                       | 1.95                |
| CAN_TX_ISR        | Interrupt    | After a message is transmitted on the CAN bus | 0.512                            | 0.005                       | 0.98                |
| CAN_TX_Task       | Thread       | When a message is added to the outgoing queue | 54ms for 36 executions           | 0.012                       | 0.80                |
| scanKeysTask      | Thread       | 20ms                                          | 20                               | 0.398                       | 1.99                |
| displayUpdateTask | Thread       | 100ms                                         | 100                              | 16.743                      | 16.74               |
| decodeTask        | Thread       | When a message is added to the incoming queue | 25.2 for 36 executions           | 0.016                       | 2.28                |


## Shared Data Structures
___

These are the shared data structures throughout the program. The table shows how each of them are protected.

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



