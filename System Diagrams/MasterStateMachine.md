# Main State Machine



```mermaid
stateDiagram-v2
    classDef badBadEvent fill: Red, color:white,font-weight:bold,stroke-width:6px,stroke:Green

    classDef lessBadEvent fill: Red,color:white,font-weight:bold,stroke-width:6px,stroke:Green
    
    classDef NoMovement font-weight:bold, stroke-width:4px,stroke:Green

    classDef Movement font-weight:bold stroke-width:6px,stroke:Yellow

    state  "Fully Mission Capable" as FMC
    
    
    [*] --> Init
    Init --> Bit_Error : IBIT Fail
    Init --> Standby : IBIT Pass
    
    Standby --> Maintenance 
    Standby --> ESTOP : BIT Fail | ESTOP
    Operation --> ESTOP : BIT Fail | ESTOP
    
    Bit_Error --> Standby : Fault Cleared
    Bit_Error --> Maintenance : ESTOP Override
    
    
    ESTOP --> Bit_Error : BIT Fail
    ESTOP --> Standby : ESTOP Cleared
    ESTOP --> Maintenance : ESTOP Override
    
    Maintenance --> Init 
    Maintenance --> Standby

    state FMC {
        Standby 
        Operation
        Ventilate

        Standby --> Operation : Call Button | Summon
        Operation --> Standby : Complete
        Standby --> Ventilate
        Ventilate --> Standby
        
    
    }
    
    state Maintenance {
        state "Manual Control" As M1
        state "Calibration" As M2
    }

    state Error {
        Bit_Error
        ESTOP
    }

    note left of Error : IBIT = Initialization Built In Test

    note left of Error : BIT = Built In Test

    Init ::: NoMovement

    Standby ::: NoMovement
    Operation ::: Movement

    Maintenance ::: Movement

    ESTOP ::: badBadEvent
    Bit_Error ::: lessBadEvent



```