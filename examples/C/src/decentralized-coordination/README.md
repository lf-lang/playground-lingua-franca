# Decentralized Coordination
The decentralized coordinator for federated Lingua Franca programs is an experimental technology with many subtleties. This coordinator is specified as a target property:

```
target {
  coordination: decentralized
}
...
federated reactor {
  a = new A()
  b = new B()
  ...
}
```

Each top-level reactor instance, such as `a` and `b` above, runs as a separate program called a "federate," possibly on different machines or in separate containers.

## The Role of the RTI
Like the (default) centralized coordinator, the RTI orchestrates the startup (and shutdown) of the federation, but unlike the centralized coordinator, the RTI plays little role during the execution of the program. Its function is limited to:

* handling requests to shut down the federation, via `lf_request_stop()` in C or `request_stop()` in Python; and
* performing runtime clock synchronization, if this is enabled.

## STA and STAA
Each federate makes its own decisions about when to advance to a tag _g_ = (_t_, _m_) and invoke reactions at that tag.  To govern these decisions, there are two key variables that a programmer must specify:

* **STA**: The "safe to advance" offset is a physical time quantity that asserts that the federate can advance to a tag _g_ = (_t_, _m_) when its local physical clock time. _T_ satisfies _T_ >= _t_ + _STA_.
* **STAA**: The "safe to assume absent" offset is a physical time quantity that asserts that the federate can assume that an input to the federate is absent at tag _g_ = (_t_, _m_) when its local physical clock time _T_ satisfies _T_ >= _t_ + _STA_ + _STAA_.

The STA is a property of a federate and is defined as a parameter for the reactor class:

```
reactor A(STA:time = <default>) { ... }
```

The STAA is associated with one or more input ports, but it is declared on reactions to those input ports:

```
reactor A {
  input in:<type>
  reaction(in) {=
    <normal operation>
  =} STAA(<time value>) {=
    <fault handler>
  =}
}
```

Any network input port that is a trigger for the reaction that declares an STAA handler will have that STAA applied to it.
If more than one STAA is declared for the same input port, the minimum time value will be the one in effect.  The `<fault handler>` is code that will be executed if an input had been previously assumed to be absent at a tag _g_, but then an input event with tag _g_ arrives.

Choosing suitable STAs and STAAs for a program is challenging and depends on many factors.  Collectively, STA and STAA are both referred to as **STP** (safe to process) offsets, but the distinction between the two is important and subtle.

## Example 1: SimpleFeedForward

Consider the [SimpleFeedForward](SimpleFeedforward.lf) example:

<img src="figures/SimpleFeedForward.png" width=300 alt="SimpleFeedForward"/>

This program mostly works with the default STA and STAA being zero. Suppose `PrintLag` is defined as follows:

```
reactor PrintLag {
  input in: int
  reaction(in) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Reaction to network input %d lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us, microstep %d.",
        in->value, lag/1000, lf_time_logical_elapsed()/1000, lf_tag().microstep);
  =}
}
```

When we execute this program, the output looks like this:

```
Fed 0 (c): Starting timestamp is: 1722799292106089000.
Fed 1 (p): Starting timestamp is: 1722799292106089000.
Fed 1 (p): ERROR: STP violation occurred in a trigger to reaction 1, and there is no handler.
**** Invoking reaction at the wrong tag!
Fed 1 (p): Reaction to network input 1 lag is 4892us at logical time 0us, microstep 1.
Fed 1 (p): Reaction to network input 2 lag is 4857us at logical time 3000000us, microstep 0.
Fed 1 (p): Reaction to network input 3 lag is 4994us at logical time 6000000us, microstep 0.
Fed 1 (p): Reaction to network input 4 lag is 5240us at logical time 9000000us, microstep 0.
...
```

Notice that the very first reaction is invoked, incorrectly, at microstep 1. This is because, with STA and STAA both being zero, the runtime system commits to the start tag (0, 0) (elapsed), sees no network input, concludes the input is absent at (0, 0), and completes execution at that tag without invoking any reactions.
When an input subsequently arrives with intended tag (0, 0), that tag cannot be assigned to it because the runtime system has already completed execution at that tag. Hence, the input gets assigned the next available tag, (0, 1), and the reaction is invoked with the stated warning.

Notice that subsequent reactions do not experience an STP violation. In fact, if we were to change the offset on the timer in the program to be bigger than 0, then the entire program would run correctly without any STP violations.  The reason for this is the `PrintLag` reactor has no events of its own except the ubiquitous `startup` event, which all reactors have.  Thus, after concluding tag (0, 0), it does not know what tag to advance to until it receives an input message. By then, there will be no need to assume the input is absent (it isn't), so it can immediately commit to the tag and execute the reactions.

The `Count` reactor also experiences no STP violations. It has no network inputs, and therefore is free to advance its tag without risk. Hence, an STA and STAA of zero work fine for that reactor.

### Adding an STAA

We can modify the `PrintLag` reactor as follows:

```
reactor PrintLag {
  input in: int
  reaction(in) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Reaction to network input %d lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us, microstep %d.",
        in->value, lag/1000, lf_time_logical_elapsed()/1000, lf_tag().microstep);
  =} STAA(10ms) {=
    lf_print("**** STP violation at PrintLag at tag " PRINTF_TAG, lf_tag().time - lf_time_start(), lf_tag().microstep);
  =}
}
```

The addition of the STAA has two effects. First, it tells the runtime system to wait before assuming the input is absent.  Second, provides a handler to invoke when the wait is insufficient and a message later arrives with a tag that is now in the past.  The output now looks like this:

```
Fed 1 (p): Reaction to network input 1 lag is 5161us at logical time 0us, microstep 0.
Fed 1 (p): Reaction to network input 2 lag is 1808us at logical time 3000000us, microstep 0.
Fed 1 (p): Reaction to network input 3 lag is 5199us at logical time 6000000us, microstep 0.
Fed 1 (p): Reaction to network input 4 lag is 902us at logical time 9000000us, microstep 0.
```

Notice that there are no STP violations and the lag is no worse than before.
The STAA of 10ms seems reasonable given that the observed latencies are reliably around 5ms. The lag is no worse than before `Count` is reliably sending messages, and hence `PrintLag` never has to actually assume the input is absent.

In fact, for this program, we could set the STAA to a very large time. For example:

```
  reaction(in) {=
    ...
  =} STP(1 day) {=
    ...
  =}
```

The lag will still not be worse than before, as long as `Count` remains reliable.

### Using STA vs. STAA

For this example, it would work equally well to set the STA rather than the STAA.
If we change the `PrintLag` reactor to read like this:

```
reactor PrintLag(STA: time = 1 day) {
  input in: int
  reaction(in) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Reaction to network input %d lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us, microstep %d.",
        in->value, lag/1000, lf_time_logical_elapsed()/1000, lf_tag().microstep);
  =}
}
```

In this case, once again there will be no STP violations and the lag will be no worse than before.
This is because the runtime system does not wait for the STA time to elapse once the network inputs become known to be present or absent.

### Warning: Timeout

Suppose that we change the `target` declaration in the above program to add a timeout:

```
target C {
  coordination: decentralized,
  timeout: 6s
}
```

With the STA set to one day, this program will nonetheless terminate correctly after receiving the input at tag (6s, 0):

```
Fed 0 (c): Starting timestamp is: 1722803307426429000.
Fed 1 (p): Starting timestamp is: 1722803307426429000.
Fed 1 (p): Environment 0: ---- Spawning 1 workers.
Fed 0 (c): Environment 0: ---- Spawning 1 workers.
Fed 1 (p): Reaction to network input 1 lag is 1439us at logical time 0us, microstep 0.
Fed 1 (p): Reaction to network input 2 lag is 3589us at logical time 3000000us, microstep 0.
Fed 1 (p): Socket from federate 0 is closed.
Fed 0 (c): Connection to the RTI closed with an EOF.
Fed 1 (p): Reaction to network input 3 lag is 1304us at logical time 6000000us, microstep 0.
---- Elapsed logical time (in nsec): 6,000,000,000
---- Elapsed physical time (in nsec): 6,001,491,000
Fed 1 (p): Connection to the RTI closed with an EOF.
---- Elapsed logical time (in nsec): 6,000,000,000
---- Elapsed physical time (in nsec): 6,002,616,000
RTI has exited. Wait for federates to exit.
All done.
```

However, suppose you set the timeout to `7s` instead of `6s`.  What happens?
The `PrintLag` federate will not exit until one day after you start it!
The reason is that after it receives an input with tag (6s, 0), it now wishes to advance its tag to the timeout tag, (7s, 0).  However, it is unknown to this federate whether there will be an input between these two tags.
Since we specified an STA of 1 day, we have told the federate that if it receives no further inputs (which it will not because the `Count` federate will have exited), then it must wait one day before advancing its tag!

## Example 2: LiveDestination

Consider the [LiveDestination](LiveDestination.lf) program:

<img src="figures/LiveDestination.png" width=300 alt="LiveDestination"/>

The `Destination` reactor is similar to `PrintLag` except that it also has a local timer.  It looks like this:

```
reactor Destination(STA: time = 1 day) {
  input in: int
  timer t(0, 500ms)
  reaction(t) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Local reaction lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us.",
        lag/1000, lf_time_logical_elapsed()/1000);
  =}
  reaction(in) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Reaction to network input %d lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us.",
        in->value, lag/1000, lf_time_logical_elapsed()/1000);
  =}
}
```

This once again has an STA of 1 day, but because inputs keep arriving, it never has to wait one day to determine that it can safely advance its tag. The output looks like this:

```
Fed 1 (p): Local reaction lag is 2344us at logical time 0us.
Fed 1 (p): Reaction to network input 1 lag is 2369us at logical time 0us.
Fed 1 (p): Local reaction lag is 2502423us at logical time 500000us.
Fed 1 (p): Local reaction lag is 2002442us at logical time 1000000us.
Fed 1 (p): Local reaction lag is 1502445us at logical time 1500000us.
Fed 1 (p): Local reaction lag is 1002452us at logical time 2000000us.
Fed 1 (p): Local reaction lag is 502456us at logical time 2500000us.
Fed 1 (p): Local reaction lag is 2462us at logical time 3000000us.
Fed 1 (p): Reaction to network input 2 lag is 2465us at logical time 3000000us.
Fed 1 (p): Local reaction lag is 2501403us at logical time 3500000us.
Fed 1 (p): Local reaction lag is 2001437us at logical time 4000000us.
Fed 1 (p): Local reaction lag is 1501446us at logical time 4500000us.
Fed 1 (p): Local reaction lag is 1001455us at logical time 5000000us.
Fed 1 (p): Local reaction lag is 501464us at logical time 5500000us.
Fed 1 (p): Local reaction lag is 1475us at logical time 6000000us.
Fed 1 (p): Reaction to network input 3 lag is 1482us at logical time 6000000us.
Fed 1 (p): Local reaction lag is 2505203us at logical time 6500000us.
...
```

Notice, however, the large lags of 2.5, 2, 1.5, etc. seconds. In fact, the local events that have tags that do not align with an input tag cannot be processed until the next network input arrives (or one day passes)!

### Using STAA vs. STA

The excessive lag can be fixed by using an STAA instead of (or in addition to) an STA.
Suppose we set the STA to zero (its default) and specify an STAA as follows:

```
  reaction(in) {=
    interval_t lag = lf_time_physical() - lf_time_logical();
    lf_print("Reaction to network input %d lag is " PRINTF_TIME "us at logical time " PRINTF_TIME "us.",
        in->value, lag/1000, lf_time_logical_elapsed()/1000);
  =} STAA(10ms) {=
    lf_print("**** STP violation at Destination at tag " PRINTF_TAG, lf_tag().time - lf_time_start(), lf_tag().microstep);
  =}
```

This program will now deliver much more reasonable lags:

```
Fed 1 (p): Local reaction lag is 4275us at logical time 0us.
Fed 1 (p): Reaction to network input 1 lag is 5161us at logical time 0us.
Fed 1 (p): Local reaction lag is 704us at logical time 500000us.
Fed 1 (p): Local reaction lag is 5034us at logical time 1000000us.
Fed 1 (p): Local reaction lag is 2744us at logical time 1500000us.
Fed 1 (p): Local reaction lag is 5040us at logical time 2000000us.
Fed 1 (p): Local reaction lag is 85us at logical time 2500000us.
Fed 1 (p): Local reaction lag is 1876us at logical time 3000000us.
Fed 1 (p): Reaction to network input 2 lag is 2015us at logical time 3000000us.
Fed 1 (p): Local reaction lag is 1259us at logical time 3500000us.
Fed 1 (p): Local reaction lag is 4263us at logical time 4000000us.
Fed 1 (p): Local reaction lag is 5035us at logical time 4500000us.
Fed 1 (p): Local reaction lag is 5033us at logical time 5000000us.
Fed 1 (p): Local reaction lag is 3078us at logical time 5500000us.
Fed 1 (p): Local reaction lag is 145us at logical time 6000000us.
Fed 1 (p): Reaction to network input 3 lag is 289us at logical time 6000000us.
Fed 1 (p): Local reaction lag is 5307us at logical time 6500000us.
```

### When can the STA be Zero?

In this case, it is safe to set the STA to zero, but only because we know that every incoming tag from the network will be logically simultaneous with a local event.
If we were to change this, for example by offsetting the timer slightly:

```
  timer t(1ms, 500ms)
```

We will now see STP violations for most network inputs.  The reason is that the `Destination` federate advances its tag to (3001ms, 0) before receiving the network input with tag (3000, 0).
The STA of zero allows this.

For this variant, a better choice would be to set the STA to 10ms and the STAA to 0.
However, now the lags on the local events will _always_ be larger than 10ms:

```
Fed 1 (p): Reaction to network input 1 lag is 5253us at logical time 0us.
Fed 1 (p): Local reaction lag is 11458us at logical time 1000us.
Fed 1 (p): Local reaction lag is 14607us at logical time 501000us.
Fed 1 (p): Local reaction lag is 12163us at logical time 1001000us.
Fed 1 (p): Local reaction lag is 14224us at logical time 1501000us.
Fed 1 (p): Local reaction lag is 15045us at logical time 2001000us.
Fed 1 (p): Local reaction lag is 12404us at logical time 2501000us.
Fed 1 (p): Reaction to network input 2 lag is 5172us at logical time 3000000us.
Fed 1 (p): Local reaction lag is 11467us at logical time 3001000us.
Fed 1 (p): Local reaction lag is 11882us at logical time 3501000us.
...
```

This is because no network inputs arrive with tags matching the local tags, so the federate always needs to wait out the STA before advancing to the local tags.

### Using After Delays

When you need a smaller lag on local events, a better solution is to use `after` delays on the network connections.  We can change the `LiveDestination` program as follows:

```
federated reactor {
  c = new Count(period = 3s)
  p = new Destination()
  c.out -> p.in after 10ms
}
```

<img src="figures/After.png" width=300 alt="After"/>

With this change, the STA and STAA can both be set to zero, and all the reported lags will be small.
However, keep in mind that although the _reported_ lag is small for the network input events, an extra 10ms has been added to their timestamps by the `after` delay.
The output looks like this for the network inputs:

```
Fed 1 (p): Reaction to network input 1 lag is 1265us at logical time 10000us.
...
Fed 1 (p): Reaction to network input 2 lag is 1458us at logical time 3010000us.
...
Fed 1 (p): Reaction to network input 3 lag is 1986us at logical time 6010000us.
...
```

The logical times at the `Count` federate were 0, 3s, 6s, etc., but here they are larger by 10ms.  This effectively adds a 10ms delay to the processing of network inputs.

### Summary

There are a few rules of thumb to guide you:

* When local event tags always match network input tags, then STA can safely set to zero, and STAA can be used to ensure that the federate keeps processing local events even if the upstream federate or the network fails.
* When local event tags do not always match network input tags, then you must decide whether to process local events quickly or network input events quickly. To process local events quickly, use an `after` delay. To process network input events quickly, use an STA.

## Example 3: MonteCarloPi

### Feedback

### Dataflow Style

Insights:

// Pi needs an STP_offset because its event queue is usually empty and otherwise it will advance to the stop time.

Need to keep producing outputs after calling lf_request_stop

