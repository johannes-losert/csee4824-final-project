# Stage Notes
## Fetch (F)
- Fetch instruction at PC from memory (or inst. cache?)
- Do branch prediction?
- Store in F/D register

## Dispatch (D)
- Take instruction in F/D register and decode it
- Attempt to allocate a Reservation Station, ROB entry, L/S Queue entry, and
physical register (unless dest-less instruction like store) for it
    - If any is unavailable, then we have it a structural hazard and must
    stall until all available
    - Otherwise, allocate each:
        1. Remove a PR number N from Free List (unless dest-less)
        2. Allocate new ROB entry at the head of the queue
        2.5. Do we need to allocate entry in L/S queue if it is a load or store?
        3. Copy current Map Table (OR is it Arch Map?) entry mapped to the
        architectural register that is the instruction's into Told of ROB entry
        4. Populate ROB entry 'T' with tag N
        5. Update Map Table entry for arch register destination, replacing old
        tag with N
        6. Mark instruction-appropriate RS as busy, and populate with
        instruction number, 'T' as tag N, and opcode tags
            a. Find opcode tags T1 and T2 by accessing Map Table (but make sure
            if opcode is itself we don't wait on our own result, i.e. neither T1
            nor T2 can be tag N)

## Issue (S)
- Check each allocated entry in reservation station if the ready bit
for both opcodes is set (Should we be using CDB instead?)
- For the first (chose some other way? do multiple?) instruction that is ready,
retrieve the opcode values from the register file and advance the instruction
to the execution stage

## Execute (X)
- Each issued functional unit performs execution on the operands provided by
the issue stage
- If a FU result is ready (stored in X/C register) advancd to complete stage
    - if multiple are ready, do they all advance?
    - do we free the FU here?
## Complete (C)
- Write result of a (just one?) functional unit
    1. Check RS entry (Or ROB entry? does it matter?) to find result PR tag
    2. Write result to that tag
    3. Update CDB to broadcast that tag to any waiting FUs
    4. Update Map Table indicating that PR N is ready

## Retire (R)
- Wait for the instruction at the head of the ROB to be completed
- If instruction is a branch and branch was mispredicted, or if exception
    - clean up each instruction behind in ROB queue:
        - Free RS entry
        - Return T to freelist
        - Set MapTable at result arch register to Told
        - Free ROB entry
        - what if inst. was store?
    - Set PC to correct value (based on branch output or just before exception?)
    - do other things? definitely
- Store head of L/S queue to data memory?
- Free ROB/LSQ entries
- Return Told to Free List
- Record T to arch map of result arch reg


# Basic Component Notes

## Map Table
- Maps architectural register names to current physical register number (never
empty, but includes 'ready' bit)
- Ready bit gets set when FU finishes execution
- Tracks either the PR number containing the current (possibly speculated) value
of each architural register (if ready bit is 1) OR tracks the PR number that
will eventually be populated with the correct value of that architectural
register at this moment.
- Operations to handle:
    - get status of architectural register 1 (PR number and ready bit)
    - get status of architectural register 2 (PR number and ready bit)
    - get status of architectural register 3 (PR number and ready bit)
    - update the PR of arch register (new dest)
        - if this happens at the same as a read, the read should return the old value
        - resets the ready bit bv
    - update the ready bit of arch register (CDB)
        - if this happens at the same time as a read, the read should return the new ready bit
        - if this happens at the same time as a update PR, the read should return the old ready bit

## Arch Map
- Mapping updated when instruction is committed/retired
- Holds last 'non-speculative' PR associated with a given architectural name
- 'Reverse map table' allows retrieving architectural register name from a
physical register number (never empty)?

## Free List
- Stores PR numbers that are available to be allocated if a newly issued
instruction needs
- PRs exit free list when allocated for a newly dispatched instruction, and
enter free list when instructions are retired
- Possible operations
    1. read from head
        - input: dequeue_en
        - output: did_dequeue, output_pr_idx
        - edge cases:
            - if empty and writing, forward
            - if empty and not writing, lower did_dequeue
    2. write to tail
        - input: input_pr_idx, enqueue_en
        - output: did_enqueue
        - edge cases:
            - if full and reading, raise did_enqueue and push
            - if full and not reading, lower did_enqueue

## Reservation Stations
### Notes
- Tracks the status of functional units, assigned to instructions that have been
 dispatched (but maybe not yet issued)
- Maps instruction number to functional unit, result PR tag, and both operand
PR tags
- Member instructions have a functional unit assigned, but are waiting on the PR
of one of their operands to be set 'ready' in the map table (and broadcast on
CDB)
- Does not hold actual values, instead references physical register

### Components
    - Compiletime array of RS entry modules and busy bits, each with index number
### Inputs
    - clock, enable, reset bits
    - [For Allocating]
        - Allocate bit (enable to try to add the given op)
        - Input operation, result PR N, op1 PR N, op2 PR N
    - [For Updating] (CDB)
        - Update bit (if high, mark ready if anywhere as opcode)
        - PR to mark ready (from CDB)

### Output
    - [Allocating]
        - done bit 
    - [Issuing] (To FU)
        - Ready bit
        - operation, op1 PR tag, op2 PR tag

## RS Entry
### Components
    - Registers: FU enum | operation | result PR Tag | Operand 1 PR
    Tag | Operand 2 PR Tag
### Inputs
    - clock, enable, reset bits
    - Write bit
    - Next::RS Entry Struct {FU enum, busy bit, result PR tag, Op 1 PR
    tag, Op 2 PR tag}
    - (alternative, different 'update' methods like 'clear', 'allocate'?)
    - should we set FU enum at compiletime? Probably
### Outputs
    - Curr::RS Entry Struct
    - valid bit?

## Load/Store Queue
- ??

## Reorder Buffer
- Tracks the state of instructions (Maps instruction number to result PR number
 (t) and old result PR number (Told))
- Structured as queue, in program order (head is oldest instruction not yet
retired, tail is youngest dispatched)
- Instructions enter when dispatched, leave when retired
- Does not hold values, now purely for control/tag management

## Common Data Bus
- Broadcasts that FU has completed instruction and stored result in PR number
- Triggers update of 'ready' bit in Map Table AND issuing of any instructions
waiting on PR number as an operand

## Register File
- Holds all physical registers (equal to number of 'architectural' registers and
the number of ROB entries)
- Usage of different physical registers for each issue eliminates WAW/WAR
hazards 'naturally'

## Misc Notes
- 'Architectural' registers are registers used by programs (e.g. R0,R1...R31)


# Advanced Feature Notes
## 2-Way Superscalar
- Processor must be able to move multiple instruction 'bundles' between stages
in a single clock cycle
- Most difficult transition will be during dispatch, when PRs are assigned
    - In a single cycle, must resolve dependencies within bundle
    - Approach:
        1. Pre-fetch PRs from free list for all instructions in bundle
        2. Detect dependencies within bundle, if no depedencies, dispatch
        normally with operand PRs from map table
        3. If instruction A in bundle depends on instruction B, don't use
        operand from map table for A, instead use result PR that was just
        fetched for B as operand PR for A
- Must expand CBD to be able to broadcast all instructions in bundle as
completing execution in a single cycle

# Questions
- How do caches work
- Difference between Arch Table and Map Table
- Do we get Told from Arch Table or Map Table (does it matter?)
- Difference between ROB and L/S queue? What is the L/S queue?
    - In general, how do load/store ops fit into pipeline
- In issue stage, if multiple instructions ready, how do I chose which one to
issue? Or can I advance more than the superscalar number of instructions during
a single issue stage?   
    - If I chose to not issue one, how do I 'remember' to do so on the next
    cycle?
- Why do we need the CDB if we can just check the map table for every allocated
RS entry's opcodes during the issue stage?
- Where do we actually convert PR tag into opcode value for the execute stage?
    - In the execute stage itself? Probably not, probably in the issue stage
- Since stages can take multiple cycles, where do we track what stage each
instruction is in? Do we do this in the ROB?
- If multiple FUs are done, how do we choose which one to complete/broadcast?
    - can we do more than superscalar width at once?

- Do we free the functional unit in the execute or complete stage?
- Where do we handle branches? How?
    - in retire stage?
- Where do we handle exceptions? How?
    - in retire stage?

- Where does branch prediction happen?
    - In IF stage?

- While rolling back, how do we undo a store instruction?
- Is the size of RS just the sum of the number of functional units?

- How to indicate to the reservation station that the ROB couldn't be allocated?
    - use 'allocated' bit based on whether or not ROB allocation succeeded? (long critical path)

- How to do instruction cache?


# OH Notes
    - Dispatch is 'Sequential'
        1. try to add to ROB
        2. based on state of ROB, try to add something to reservation station
        3. etc.