# Python Scripts

These scripts provide support for additional functionality
within Delphyne.

## Replaying a simulation

A simulation must first be recorded or _logged_. To log a simulation
means to capture all messages (along with any referenced resources)
sent from the backend to the frontend.

A simulation runner can be easily configured to produce such
recordings while the simulation takes place. All examples permit
logging-related flags to enable and route these recordings to specific
locations within your local filesystem.

Then, logs have to be reproduced in a way that's transparent for the
visualization frontend. And that's what the `delphyne-replay` script is
for.

As an example, first run a demo with logging enabled as follows:

```sh
delphyne-gazoo -d 30 -l -f /tmp/test.log
```

After simulation ends, logs will have been written to `/tmp/test.log`.

Then, replay them using:

```sh
delphyne-replay /tmp/test.log
```

You should be able to see the same simulation being reproduced.

For further reference on logging support, check [the Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ/view#heading=h.yv2cdz8gs3l9).
