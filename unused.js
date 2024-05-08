// t is parameter along spline
// n is number of control points
// k is degree + 1
// J is list of knot points (size n+k)
// P is list of control points (size n)
function evaluate_B_spline(spline, t) {
    // console.log(J);
    let pt_dim = spline.pt_dim;
    let k = spline.k;
    let n = spline.n;
    let basis_matrix = matrix_create_NxM_all_zeroes(k, n + k - 1);
    // m=1:
    for (let i = 0; i < n + k - 2; i++) {
        if (t >= spline.J[i] && t < spline.J[i+1]) {
            basis_matrix[0][i] = 1;
        }
    }

    // m=2 to m=k: fill in basis function values
    for (let m = 2; m <= k; m++) {
        for (let i = 0; i < n + k - m; i++) {
            // let c1 = (t - spline.J[i]) / (spline.J[i+k-1] - spline.J[i]);
            let top = t - spline.J[i];
            if (Math.abs(top) < 0.01) {
                top = 0;
            }
            let c1 = top / (k - 1);
            let c2 = (spline.J[i + k] - t) / (k - 1);
            // let c2 = (spline.J[i + k] - t) / (spline.J[i + k] - spline.J[i + 1]);
            // console.log(spline.J[i + k], spline.J[i + 1]);
            console.log(m, i, c1, c2);
            basis_matrix[m-1][i] = c1 * basis_matrix[m-2][i] + c2 * basis_matrix[m-2][i+1];
        }
    }

    console.log(basis_matrix);

    // use final row of basis functions to compute weighted sum
    return compute_weighted_sum(pt_dim, n, P, basis_matrix[basis_matrix.length - 1]);
}

function evaluate_B_spline(spline, t) {
    let d = spline.d;
    let J = spline.J;
    let P = spline.P;
    let pt_dim = spline.pt_dim;

    // compute index of knot interval containing t
    let idx = d - 1;
    for (let i = d - 1; i < J.length - 1; i++) {
        if (J[i] <= t && t < J[i+1]) {
            idx = i;
            break;
        }
    }

    // set up "new" / simulated control points
    let new_pts = [];
    for (let i = 0; i <= d; i++) {
        new_pts.push(P[i + idx - d]);
    }
    console.log(new_pts);

    for (let r = 1; r <= d; r++) {
        for (let i = d; i >= r; i--) {
            let alpha = (t - J[i + idx - d]) / (J[i + 1 + idx - r] - J[i + idx - d]);
            // console.log(new_pts[i-1], new_pts[i]);
            // new_pts[i] = compute_weighted_sum(pt_dim, 2, [new_pts[i-1], new_pts[i]], [1.0 - alpha, alpha]);
            new_pts[i] = [(1.0 - alpha) * new_pts[i-1][0] + alpha * new_pts[i-1][0], (1.0 - alpha) * new_pts[i-1][1] + alpha * new_pts[i][1]];
        }
    }

    return new_pts[d];
}



// draw an approximation of the specified B-spline using small links
function draw_B_spline(spline, res=0.01) {

    let t = 0;
    let t_max = spline.n - 1;
    // let t_max = 1;

    let p_old = evaluate_B_spline(spline, t);
    let p_new = [];

    t += res;

    while (t <= t_max) {
        p_new = evaluate_B_spline(spline, t);
        engine.draw_debug_line(p_old, p_new);
        p_old = p_new;
        t += res;
    }
}

function get_spline_from_points(d, P) {
    let n = P.length;
    let k = d + 1;
    let J = [];
    for (let i = 0; i < k; i++) {
        J.push(-k/2.0);
    }
    for (let i = 0; i <= n + k; i++) {
        J.push(-k/2.0 + i);
    }
    for (let i = 0; i < k; i++) {
        J.push(-k/2.0 + n + k - 1);
    }
    console.log(J);
    // used for padding
    // J.push(J[J.length - 1]);
    return {
        pt_dim: P[0].length,
        n: n,
        k: k,
        d: d,
        J: J,
        P: P,
    };
}