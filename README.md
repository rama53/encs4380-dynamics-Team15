# Dynamic Characteristics Toolkit — Team 15

An interactive web toolkit for exploring sensor dynamic characteristics, developed as a Homework for ENCS4380.
## Live Website

[Open the Dynamic Characteristics Toolkit](https://rama53.github.io/encs4380-dynamics-Team15/)

## Team Members and Responsibilities

| Team Member | Module |
|---|---|
| Rama Anabtawi | Module 1: First-Order Time Response |
| Rahaf Doufash | Module 2: First-Order Frequency Response |
| Omar Shobaki | Module 3: Second-Order Response |
| Itidal Fares | Module 4: Guess the Sensor Challenge |

## Modules

1. **First-Order Time Response:** Explore step and ramp responses, the time constant, steady-state error, and the error-fraction plot.
2. **First-Order Frequency Response:** Explore sinusoidal responses, amplitude ratio, phase lag, Bode plots, and dynamic error.
3. **Second-Order Response:** Explore damping, response metrics, poles, and the state-space model.
4. **Guess the Sensor Challenge:** Estimate hidden sensor parameters from response plots and check your predictions.

### How Module 4 works

- Each round picks a hidden random sensor: first order (random K and τ) or second order (random ζ and ωn).
- First-order sensors get a unit step or, sometimes, a sine input at a stated frequency. Second-order sensors get a unit step.
- The page plots a noisy response and shows t and y under the pointer.
- A two-level hint button points to the method: the 63.2% point and log method for first order, amplitude and phase for the sine case, and overshoot and peak time for second order.
- On submit, the page reveals the true values and overlays the true curve and the guess curve.
- Each parameter earns full marks within 10% error, decaying linearly to zero at 50% error. A running score is kept for the session.
- "New sensor" starts another round.

## AI Assistance
 **Module 1:**
 - Helping design the layout and the cards
 - Helping write the code
 - Explained some unclear details in the understanding of the homework
   
**Module 2:**

- Helping write and organize the HTML, CSS, and JavaScript code
- Helping design and format the webpage
- Explaining the frequency response equations and calculations
- Helping debug and test the interactive features and plots
  
**Module 3:**

- Help with writing the code and formatting the webpage
- Explanations of the parameters and what they mean
- Explanations of the Module 3 requirements

**Module 4:**

- Helping write the code and match the design of Module 1,2 & 3
- Helping plan the random sensors, the noise, and the scoring
- Explaining the equations used for the first-order and second-order responses
- Helping update the home page and the README

## Modules Screenshots

### Module 1:
#### Sensor Parameters sliders
<img width="781" height="266" alt="image" src="https://github.com/user-attachments/assets/95ba50e3-403e-41ac-84ce-fac995a397be" />

#### Step response plotting
<img width="737" height="418" alt="image" src="https://github.com/user-attachments/assets/f9c831c8-a3cd-4cfb-827e-442ddcf613b2" />

#### Final values and error fraction plot
<img width="763" height="442" alt="image" src="https://github.com/user-attachments/assets/2382e831-b2e8-4f5c-8391-eb8cba9b85f8" />

#### Predict then check and validation check
<img width="688" height="446" alt="image" src="https://github.com/user-attachments/assets/54c48cf1-ae1f-4551-911c-82e52ec73fd6" />

### Module 2:
#### Sensor Parameters and Results
<img width="873" height="449" alt="Screenshot 2026-11-10 195638" src="https://github.com/user-attachments/assets/cc7ee44c-d8f8-4f2f-a1a1-3df0217d0fb8" />

#### Input and Output Sine Waves
<img width="867" height="506" alt="Screenshot 2026-11-10 195649" src="https://github.com/user-attachments/assets/a5f8915c-7525-46f2-9df4-a93e4843a8bd" />

#### Bode Magnitude Plot
<img width="868" height="468" alt="Screenshot 2026-11-10 195659" src="https://github.com/user-attachments/assets/dea755c0-22dc-4bbe-9f96-888359d5aad6" />

### Bode Phase Plot
<img width="876" height="501" alt="Screenshot 2026-11-10 195710" src="https://github.com/user-attachments/assets/b8e79082-bf8c-4445-bc3a-a8873e0b3e8f" />

#### Dynamic Error and Predict-then-Check
<img width="865" height="425" alt="Screenshot 2026-11-10 195718" src="https://github.com/user-attachments/assets/741315c5-6be8-4028-a33f-64b0166cbfb1" />

#### Validation Tests
<img width="863" height="405" alt="Screenshot 2026-11-10 195728" src="https://github.com/user-attachments/assets/f61f28e7-63ad-45ed-8713-7ecbb24666f7" />

### Module 3:
#### Mass spring damper controls
<img width="1168" height="681" alt="image" src="https://github.com/user-attachments/assets/65c97378-73de-4a70-9fce-fb3d05fee651" />

#### Predict then check and system parameters
<img width="1162" height="859" alt="image" src="https://github.com/user-attachments/assets/dd490cb4-4dc9-46cc-bed2-6737edd31d51" />

#### Unit force step response
<img width="1080" height="859" alt="image" src="https://github.com/user-attachments/assets/f741dc0c-3611-4a28-babe-7905607221a9" />

#### Response measurements
<img width="1178" height="567" alt="image" src="https://github.com/user-attachments/assets/5e192f2c-0330-4ccc-bc36-8719e736a31f" />

#### Poles and eigenvalues
<img width="1161" height="875" alt="image" src="https://github.com/user-attachments/assets/53d93b9d-20ec-44b5-891b-0c2a58e8f214" />

#### State space model
<img width="1098" height="456" alt="image" src="https://github.com/user-attachments/assets/993d6378-d931-4761-9133-71f7771f651d" />

#### Position velocity phase plane
<img width="1086" height="750" alt="image" src="https://github.com/user-attachments/assets/3e227334-cd60-4f43-9b4b-7fd2359b3419" />

#### Validation checks
<img width="1103" height="456" alt="image" src="https://github.com/user-attachments/assets/847904f9-d3fb-4cc4-9baf-6a16aa48400f" />

### Module 4:
#### First-order sensor: noisy step response
<img width="722" height="683" alt="Screenshot 2026-11-11 230339" src="https://github.com/user-attachments/assets/a33a589c-a71e-40d3-a7e1-39465739dc20" />

#### First-order sensor: sine input
<img width="732" height="738" alt="Screenshot 2026-11-11 230756" src="https://github.com/user-attachments/assets/e8b28fad-062f-4a11-b776-94c170333e10" />

#### Second-order sensor response
<img width="731" height="708" alt="Screenshot 2026-11-11 230413" src="https://github.com/user-attachments/assets/56b69ebb-9152-48c8-a18d-7a436ea74d6b" />

#### Estimate form and hint
<img width="732" height="738" alt="Screenshot 2026-11-11 230756" src="https://github.com/user-attachments/assets/24722c74-12fb-4ae8-9b8e-a120c681a053" />

#### Reveal with true and guess curves and score table
<img width="740" height="897" alt="Screenshot 2026-11-11 230205" src="https://github.com/user-attachments/assets/7c73a02a-d8c9-4634-bdcd-8babcde4b032" />





