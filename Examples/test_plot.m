rng = linspace(-1, 1, 30);
[X, Y, Z] = meshgrid(rng, rng, rng);
V = X.^2+Y.^2+Z.^2;
[f, v, c] = isosurface(X, Y, Z, V, 0.5, X);
figure
patch([-1 -1 1 1], [1 -1 -1 1], [0 0 0 0], 'green');
x = patch('Faces', f, 'Vertices', v, 'FaceVertexCData', c, 'FaceColor', 'interp', 'EdgeColor', 'none');
alpha(x, 0.1);