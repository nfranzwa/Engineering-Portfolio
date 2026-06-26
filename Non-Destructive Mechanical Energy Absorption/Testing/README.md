# Testing

MTS tensile-tester runs on a 4×4 chiral structure built from the snap-through nodes.

The structure is **auxetic** — its negative Poisson's ratio means tensile loading stretches it perpendicular to the applied stress, and compressive loading shrinks it. In the plots below (tensile only), the snap-through mechanism is visible as the energy wells climb to a period of high resistance that triggers a snap. Energy is absorbed without plastic deformation.

Test video:

https://github.com/user-attachments/assets/e0c1d725-a7ef-44b0-91d4-17e20f60732f

![Specimen on the MTS](https://github.com/user-attachments/assets/f351ea4f-a297-4d3a-843e-377e794b7f44)

## Results

Force vs. displacement, single tensile pull:

![F vs d, single pull](https://github.com/user-attachments/assets/4e73ddda-9d30-4a54-9194-e71d58a09052)

Cyclic loading — the sinusoid should be cleaner. The fluctuations come from bearing slip and small assembly errors building up over cycles.

![Cyclic loading](https://github.com/user-attachments/assets/72336c28-10c5-4c27-8f8d-e014e85ab07b)

![Cyclic loading detail](https://github.com/user-attachments/assets/9112e18e-f74c-467f-8974-61a53bcad566)

## Open issue — forward / reverse bias

The snap-through mechanism is bidirectional and should produce peaks and troughs symmetric in magnitude. In the forward (tensile) direction the structure peaks higher than its troughs; in the backward (compressive) direction the opposite. The likely root cause is cheap bearings + sub-optimal assembly producing small slips that bias each cycle.

![Forward / reverse bias](https://github.com/user-attachments/assets/4bac1a4d-4645-47fc-ba15-fe018ffc428b)

## Raw data

See [`Data/`](Data) for the raw MTS DAQ exports.
