# maze_solving_neato
$$
\mathbf A = \begin{bmatrix}  \mathbf{1} & \mathbf{X} & \mathbf{Y} \end{bmatrix} \\
$$

EQ 1

$$
\mathbf Z = \mathbf A \mathbf C
$$

EQ 2

$$
\begin{align}
x_{basis} = \begin{bmatrix} \frac{1}{\sqrt{1+b^2}} \cr 0 \cr \frac{b}{\sqrt{1+b^2}} \end{bmatrix} \qquad (1) \\
y_{basis} = \begin{bmatrix} 0 \cr \frac{1}{\sqrt{1+c^2}} \cr \frac{c}{\sqrt{1+c^2}} \end{bmatrix} \qquad (2) 
\end{align}
$$

EQ 3

$$
\mathbf P = \begin{bmatrix}  \mathbf{X'} \cr \mathbf{Y'} \cr \bf{Z'} \end{bmatrix}
$$

EQ 4

$$
\begin{align}
\mathbf {x}_{coeffs} = \mathbf{x}_{basis}'\mathbf{P} \qquad (1) \\
\mathbf {y}_{coeffs} = \mathbf{y}_{basis}'\mathbf{P} \qquad (2)
\end{align}
$$

EQ 5

$$
\begin{align}
\mathbf{x}_{range} = \mathrm{norm} (\mathbf{x}_{basis}(x_{max,c} - x_{min,c})) \qquad (1) \\
\mathbf{y}_{range} = \mathrm{norm} (\mathbf{y}_{basis}(y_{max,c} - y_{min,c})) \qquad (2)
\end{align}
$$

EQ 6

$$
y_{pixels} = x_{pixels} * (y_{range} / x_{range}) 
$$

EQ 7

$$
\begin{align}
AT_x = ( \mathbf {x}_{coeffs} - x_{min,c} ) / (x_{max,c} - x_{min,c}) x_{pixels} \qquad (1) \\
AT_y = ( \mathbf {y}_{coeffs} - y_{min,c} ) / (y_{max,c} - y_{min,c}) y_{pixels} \qquad (2)
\end{align}
$$

EQ 8
