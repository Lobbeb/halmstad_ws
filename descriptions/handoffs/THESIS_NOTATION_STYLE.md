# Thesis Notation Style

This document defines the mathematical notation style to use consistently across the thesis.

## Goal

Use one notation system throughout the thesis so equations stay readable when the text involves:

- Gazebo positions and orientations
- OMNeT-mapped positions and mobility states
- communication metrics such as distance, RSSI, SNIR, and PER
- transformations, coordinate mappings, and estimators

The main principle is:

- scalars are plain
- vectors are bold lowercase
- matrices are bold uppercase
- estimates use hats
- labels such as `G`, `u`, and `g` are upright

## Recommended Preamble Macros

```latex
\usepackage{amsmath}
\usepackage{bm}
\usepackage{mathtools}

\newcommand{\vect}[1]{\bm{#1}}
\newcommand{\mat}[1]{\mathbf{#1}}
\newcommand{\set}[1]{\mathcal{#1}}
```

These macros should be preferred over ad hoc formatting inside each equation.

## Symbol Classes

### Scalars

Use plain math italic for scalar quantities.

Examples:

```latex
t_k,\quad d_k,\quad \psi_k,\quad f_c,\quad P_{\mathrm{tx}},\quad P_{\mathrm{noise}}
```

Use this for:

- time
- distance
- yaw angle
- carrier frequency
- transmit power
- noise power
- RSSI, SNIR, and PER values

### Vectors

Use bold lowercase for vectors.

Examples:

```latex
\vect{p}_k,\quad \vect{v}_k,\quad \vect{o},\quad \vect{m}_k
```

Use this for:

- position vectors
- velocity vectors
- offset vectors
- metric vectors or tuples when represented compactly

Do not use arrow notation such as `\vec{p}` for thesis vectors.

Do not use hats to mean "vector". Hats are reserved for estimates.

### Matrices

Use bold uppercase for matrices.

Examples:

```latex
\mat{S},\quad \mat{R},\quad \mat{T}
```

Use this for:

- scaling matrices
- rotation matrices
- transformation matrices

### Sets

Use calligraphic symbols for sets, collections, or indexed groups.

Examples:

```latex
\set{M}_k,\quad \set{S}_k,\quad \set{W}_k
```

Use this for:

- set of tracked models
- snapshot set
- event window

### Estimates

Use a hat only when a quantity is an estimate or inferred value.

Examples:

```latex
\hat{d}_k,\quad \hat{\vect{p}}_k
```

Do not use hats for ordinary vectors or mapped coordinates.

## Labels And Indices

Use upright text for labels that name a system, agent, or category rather than a variable.

Examples:

```latex
\vect{p}_k^{\mathrm{G}},\quad
\vect{p}_{\mathrm{u},k},\quad
\vect{p}_{\mathrm{g},k},\quad
P_{\mathrm{tx}},\quad
d_{\min}
```

Recommended label meanings:

- `\mathrm{G}` for Gazebo quantities
- `\mathrm{u}` for UAV
- `\mathrm{g}` for UGV
- `\mathrm{map}` for mapped quantities if needed
- `\min` only through built-in operators such as `\max` or `\min`, not as a handwritten variable name

## Pose Representation

For most of the thesis, pose should be written as position plus yaw:

```latex
(\vect{p}_k, \psi_k)
```

This is preferable when discussing geometry, mobility, and coordinate mapping.

If a compact state vector is needed for derivations, define it explicitly:

```latex
\vect{x}_k =
\begin{bmatrix}
x_k & y_k & z_k & \psi_k
\end{bmatrix}^{\top}
```

Do not switch between these two forms without defining the change clearly.

## Operators And Functions

Use standard math operators in upright form.

Preferred forms:

```latex
\max,\quad \min,\quad \sin,\quad \cos,\quad \log,\quad \exp
```

For functions that are not predefined, use `\operatorname{...}`.

Examples:

```latex
\operatorname{atan2}(y,x),\quad
\operatorname{wrap}_{[-\pi,\pi]}(\psi)
```

Do not write textual operators in plain italics.

Bad:

```latex
atan2(y,x)
```

Good:

```latex
\operatorname{atan2}(y,x)
```

## Norms, Absolute Values, And Delimiters

Use:

```latex
\left\| \vect{p}_{\mathrm{u},k} - \vect{p}_{\mathrm{g},k} \right\|_2
```

for norms, and:

```latex
\left| x \right|
```

for absolute values.

General rule:

- use `\left` and `\right` for automatically sized delimiters
- if they become too large, use manual sizes such as `\big`, `\Big`, `\bigg`, `\Bigg`

Examples:

```latex
\left( \frac{a}{b} \right),\quad
\left[ x+y \right],\quad
\left\{ x \in \mathbb{R} \mid x > 0 \right\}
```

## Piecewise Definitions

Use `cases` or `dcases` for piecewise equations.

Preferred style:

```latex
\[
\vect{v}_k =
\begin{cases}
\dfrac{\vect{p}_k - \vect{p}_{k-1}}{t_k - t_{k-1}}, & \text{if } \vect{p}_k \neq \vect{p}_{k-1} \text{ and } t_k > t_{k-1},\\
\vect{0}, & \text{otherwise.}
\end{cases}
\]
```

For larger operators inside cases, `dcases` from `mathtools` is often better.

Example:

```latex
\[
\mathrm{PER}_k =
\begin{dcases}
\frac{1}{|\set{W}_k|} \sum_{e \in \set{W}_k} \delta_e, & |\set{W}_k| > 0,\\
0, & |\set{W}_k| = 0.
\end{dcases}
\]
```

## Units

Units should be upright, not italic.

Examples:

```latex
0.1\,\text{m},\quad 2.4\,\text{GHz},\quad 13\,\text{dBm}
```

If the thesis already uses `siunitx`, prefer that package consistently. Otherwise use `\text{...}` for units in math mode.

## Equality Style

Use one symbol per line unless alignment genuinely improves readability.

Prefer:

```latex
\[
\vect{p}_{m,k} = \mat{S}\vect{p}_{m,k}^{\mathrm{G}} + \vect{o}
\]
```

Use `align` when multiple related equations need alignment:

```latex
\begin{align}
x_{m,k} &= x_{m,k}^{\mathrm{G}} + 50,\\
y_{m,k} &= -y_{m,k}^{\mathrm{G}} + 50,\\
z_{m,k} &= z_{m,k}^{\mathrm{G}}.
\end{align}
```

Do not use `align` for a single short equation unless there is a good reason.

## Thesis-Specific Preferred Symbols

These are the recommended standard symbols for this thesis:

- `\vect{p}` for position
- `\vect{v}` for velocity
- `\vect{o}` for offset
- `\mat{S}` for scale or mapping matrix
- `\psi` for yaw
- `d` for distance
- `L^{\mathrm{FS}}` for free-space path loss
- `\set{M}` for a model set
- `\set{S}` for a snapshot set
- `\set{W}` for the PER event window

Specific examples:

```latex
\vect{p}_{m,k}^{\mathrm{G}}
```

Gazebo position of model `m` at time index `k`

```latex
\vect{p}_{m,k}
```

OMNeT-mapped position of model `m` at time index `k`

```latex
\vect{p}_{\mathrm{u},k},\quad \vect{p}_{\mathrm{g},k}
```

UAV and UGV positions

```latex
\psi_k^{\mathrm{G}},\quad \psi_k
```

Gazebo yaw and mapped yaw

```latex
d_k,\quad d_k^{\mathrm{radio}}
```

geometric distance and RSSI-inverted radio distance estimate

## Consistency Rules

- Do not mix `\vec`, `\mathbf`, and `\bm` for the same kind of object
- Do not use hats unless the quantity is an estimate
- Do not use italic letters for label-style indices such as `G`, `u`, `g`, `tx`, or `noise`
- Do not change the meaning of a symbol later in the thesis
- Define any overloaded symbol clearly before using it

## Quick Examples

Coordinate mapping:

```latex
\[
\vect{p}_{m,k} = \mat{S}\vect{p}_{m,k}^{\mathrm{G}} + \vect{o}
\]
```

Distance:

```latex
\[
d_k = \max\left(d_{\min}, \left\| \vect{p}_{\mathrm{u},k} - \vect{p}_{\mathrm{g},k} \right\|_2\right)
\]
```

Path loss:

```latex
\[
L_k^{\mathrm{FS}} = 20\log_{10}(d_k) + 20\log_{10}(f_c) - 147.55
\]
```

Packet error rate:

```latex
\[
\mathrm{PER}_k =
\begin{dcases}
\frac{1}{|\set{W}_k|} \sum_{e \in \set{W}_k} \delta_e, & |\set{W}_k| > 0,\\
0, & |\set{W}_k| = 0.
\end{dcases}
\]
```
