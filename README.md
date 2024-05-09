# sCube-software


High level description:

-- 1 Base station, N cubes

-- Communication Interface: 

-- Base station to cubes: Radio (915 MHz)

-- Cube to cube: NFMI

-- The cubes communicate with one another by sending the following information:

    -- Cube ID
    
    -- Face ID through which it transmits the signal

-- Each cube maintains a table that is updated once it gets packets from the neighboring cubes

-- Each entry contains ID of the neighboring node
   
    -- Face ID of the neighboring node
   
    -- Face ID of its own

-- The Base Station periodically sends ‘POLL’ packets to all the cubes in its communication range and in the same network

-- The cubes on receiving the POLL packets from the BSS, sends the following information to the base station:

    -- Table with cube ID and face ID as the entries
    
    -- Acceleration along three axes
    
    -- Battery information

-- From the accelerometer data that the BSS got from the cubes, it outputs the orientation of the cube


Code Structure:

1. Base station (BS_sCube_V2.ino)

-- Csrfheader:
    
    Structure for realizing packet header
    
    Contains the information about source, destination and type of packet (BS to Cube, Cube to BS and Cube to Cube)

-- CsrfPacket:

    Structure for realizing packet 
    
    Contains csrfheader and the data

-- NeighborTableEntry:

    Structure for storing the information from the cubes
    
    The cubes send the localized information that it obtained by communicating with its neighbor
    
    Cube ID, Face ID of neighbor and Its own face ID

-- Statusentry:

    Structure for storing other information received from the cubes, such as, battery information, acceleration values along three axes

-- sqrWaveA2:

    Function for generating square wave signal
    
    Used for sending data over NFMI interface

-- Setup:

    Function for initialization of parameters
    
    Pins are initialized
    
    Radio is initialized

-- Void Loop():

    Create the POLL packet to be sent
    
    Header info: (Source: BS, Dest: 0 (Broadcast), type: Poll packet)
    
    Transmit the POLL packet
    
    Put radio in receive mode
    
    Receive packet from the cube
    
    Extract information (such as, Cube_ID) from the header of the received packet
    
    Update the table using the information received (Localized neighborhood information of the cube)
    
    From the acceleration data from the cubes, it computes their orientations
    
    Wait till the next time slot


2. sCube (Block_sCube_V2.ino)

-- Csrfheader:
    
    Structure for realizing packet header
    
    Contains the information about source, destination and type of packet

-- CsrfPacket:

    Structure for realizing packet 
    
    Contains csrfheader and the data

-- NeighborTableEntry:

    Structure for storing the information from the neighboring cubes
    
    Has the localized information that it obtained by communicating with its neighbor

-- Void initRadio():

    Function for radio initialization

-- Setup:
 
    Function for initialization of parameters
    
    Pins are initialized
    
    Radio is initialized

-- void NFMI_RX_INT_enable()

    Used for enabling receive mode for NFMI

-- void NFMI_RX_INT_disable()

    Used for disabling receive mode for NFMI

-- void nf_send_bit(box_id, face_id):

    Convert box_id and face_id to 4 digit and 3-digit binary
    
    Generate the transmitting signal using the box_id and face_id

-- Void Loop():

    Wait for packets to be received
    
    Check if the packet is a POLL packet from base station
    
    Check if the packet is destined to itself
    
    If yes:
    
        Include its table in the outgoing packet
        
        Put its accelerometer data in the outgoing packet
        
        Enable NFMI
        
        Transmit proximity signal containing the information of BLOCK_ID and FACE_ID 
    
    Else:
     
        Set NFMI to receive mode
        
        Receive the packets from the neighboring cubes
        
        Update its table using the FACE_ID information using the neighbor (fillNeighbor function)



Sequence of events executed in cube 𝑖:

A. Initialize the radio and accelerometer. Set the input/output pins

B. Turn On the Blue LED after the required modules are initialized

C. Wait till it receives any POLL from the base station

D. When a cube 𝑖 receives a ‘Poll’ from the base station:

    1. Checks if the received poll is for itself
    
    2. If the poll is for itself:
    
          i. Sends a poll response to the base station with the following information:
          
                Own cube ID
                
                Destination (Base station ID)
                
                Flag indicating that it is a poll response 
                
                Table with the information received via all its NFMI coils
                
                Accelerometer readings along 3-axes
                
                Battery charge
                
                Time elapsed from the poll; time elapsed from the start of the cube
          
          ii. Put the NFMI interface in the transmit mode
          
          iii. Sends NFMI signals along all the 6 faces in a TDMA manner
    
    3. Else (poll is for some other cubes):
    
          i. Put the NFMI interfaces in receiving mode
          
          ii. Listens across all its 6 faces in a TDMA manner
          
          iii. If there is any signal received across any of its faces:
          
                Save the face id through which it received signal and the timestamp
                
                Update the table entry (Face ID of cube 𝑖, Neighboring cube ID, Neighboring cube’s face ID)

Arduino Library List:

1.	Adafruit LIS3DH Version: 1.0.0
2.	Adafruit Unified Sensor Version: 1.1.5
3.	Adafruit SPI Flash Version: 3.10.0
4.	Adafruit RFM69 LowPowerLab Version: 1.0.0
5.	WSN_RFM69 Version: 1.0.0
6.	SPIFlash_LowPowerLab Version: 101.1.3


