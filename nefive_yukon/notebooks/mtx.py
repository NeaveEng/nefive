def _dot(a, b):
    s = 0
    for i in range(0, len(b)):
        s += a[i] * b[i]
    return s

def mul(A, b):
    return [_dot(row, b) for row in A]

def _height(A):
    return len(A)

def _width(A):
    return len(A[0])

def _max_magnitude_row(A, column):
    first_row = column

    max_row = first_row
    max_value = A[first_row][column]

    for i in range(first_row+1, _height(A)):
        if abs(A[i][column]) > abs(max_value):
            max_row = i
            max_value = A[i][column]

    return max_row

def _subtract_rows_below(A, row, column, pivot_value):
    # Fill in factors where entries will become zero.
    for i in range(row+1, _height(A)):
        A[i][column] /= pivot_value

    for i in range(row+1, _height(A)):
        for j in range(column+1, _width(A)):
            A[i][j] -= (A[i][column] * A[row][j])

def lu(A):
    P = None
    for pivot_column in range(0, _width(A)-1):
        pivot_row = pivot_column
        max_row = _max_magnitude_row(A, pivot_column)
        pivot_value = A[max_row][pivot_column]

        if max_row != pivot_row:
            A[pivot_row], A[max_row] = A[max_row], A[pivot_row]

            if P is None:
                P = list(range(0, _height(A)))

            P[pivot_row], P[max_row] = P[max_row], P[pivot_row]

        _subtract_rows_below(A, pivot_row, pivot_column, pivot_value)
    return P,A

def _forward_substitute(A, b):
    for i in range(1, len(b)):
        for j in range(0, i):
            b[i] = b[i] - (b[j] * A[i][j])

def _backward_substitute(A, b):
    size = len(b)
    b[-1] = b[-1] / A[-1][-1]
    for i in range(size-2, -1, -1):
        for j in range(i+1, size):
            b[i] = b[i] - (b[j] * A[i][j])
        b[i] = b[i] / A[i][i]

def solve(PLU, b):
    P, LU = PLU

    x = b[:] if P is None else [b[i] for i in P]  # permutation
    _forward_substitute(LU, x)
    _backward_substitute(LU, x)
    return x