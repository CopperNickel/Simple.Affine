using System.Diagnostics.CodeAnalysis;

namespace Simple.Affine;

/// <summary>
/// Represents a 2D affine transformation matrix.
/// </summary>
public readonly struct TransformationMatrix : IEquatable<TransformationMatrix> {
  #region Properties

  /// <summary>
  /// Gets the value of the (1, 1) element of the transformation matrix.
  /// </summary>
  public double A11 { get; }
  /// <summary>
  /// Gets the value of the (1, 2) element of the transformation matrix.
  /// </summary>
  public double A12 { get; }
  /// <summary>
  /// Gets the value of the (1, 3) element of the transformation matrix.
  /// </summary>
  public double A13 { get; }
  /// <summary>
  /// Gets the value of the (2, 1) element of the transformation matrix.
  /// </summary>
  public double A21 { get; }
  /// <summary>
  /// Gets the value of the (2, 2) element of the transformation matrix.
  /// </summary>
  public double A22 { get; }
  /// <summary>
  /// Gets the value of the (2, 3) element of the transformation matrix.
  /// </summary>
  public double A23 { get; }

  /// <summary>
  /// Determines the determinant of the transformation matrix.
  /// </summary>
  public double Determinant => A11 * A22 - A12 * A21;

  public double TranslationX => A13;

  public double TranslationY => A23;

  public double Angle => Math.Atan2(A21, A11);

  public double ScaleX => Math.Sqrt(A11 * A11 + A22 * A22);

  public double ScaleY => Determinant / ScaleX;

  public double ShearFactor => (A11 * A12 + A21 * A22) / Determinant;

  #endregion Properties

  #region Create

  /// <summary>
  /// Gets the identity transformation matrix.
  /// </summary>
  public static TransformationMatrix Identity { get; } = new TransformationMatrix(1, 0, 0, 0, 1, 0);

  /// <summary>
  /// Gets a transformation matrix that represents a mirror transformation.
  /// </summary>
  public static TransformationMatrix Mirror { get; } = new TransformationMatrix(0, 1, 0, 1, 0, 0);

  /// <summary>
  /// Gets a transformation matrix that represents an error state, with all elements set to NaN.
  /// </summary>
  public static TransformationMatrix Error { get; } = new TransformationMatrix(double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN);

  /// <summary>
  /// Creates a new instance of the <see cref="TransformationMatrix"/> struct with the specified values.
  /// </summary>
  /// <param name="a11">The value for the (1, 1) element.</param>
  /// <param name="a12">The value for the (1, 2) element.</param>
  /// <param name="a13">The value for the (1, 3) element.</param>
  /// <param name="a21">The value for the (2, 1) element.</param>
  /// <param name="a22">The value for the (2, 2) element.</param>
  /// <param name="a23">The value for the (2, 3) element.</param>
  public TransformationMatrix(double a11, double a12, double a13, double a21, double a22, double a23) {
    A11 = a11;
    A12 = a12;
    A13 = a13;
    A21 = a21;
    A22 = a22;
    A23 = a23;
  }

  /// <summary>
  /// Creates a transformation matrix that represents a rotation by the specified angle (in radians).
  /// </summary>
  /// <param name="angle">The angle of rotation in radians.</param>
  /// <returns>A transformation matrix representing the rotation.</returns>
  public static TransformationMatrix Rotate(double angle) {
    var (sin, cos) = Math.SinCos(angle);

    return new(cos, -sin, 0, sin, cos, 0);
  }

  /// <summary>
  /// Creates a transformation matrix that represents a translation by the specified deltaX and deltaY values.
  /// </summary>
  /// <param name="deltaX">The amount of translation in the X direction.</param>
  /// <param name="deltaY">The amount of translation in the Y direction.</param>
  /// <returns>A transformation matrix representing the translation.</returns>
  public static TransformationMatrix Translate(double deltaX, double deltaY) {
    return new(1, 0, deltaX, 0, 1, deltaY);
  }

  /// <summary>
  /// Creates a transformation matrix that represents scaling by the specified factorX and factorY values.
  /// </summary>
  /// <param name="factorX">The scale factor in the X direction.</param>
  /// <param name="factorY">The scale factor in the Y direction.</param>
  /// <returns>A transformation matrix representing the scaling.</returns>
  public static TransformationMatrix Scale(double factorX, double factorY) {
    return new(factorX, 0, 0, 0, factorY, 0);
  }

  /// <summary>
  /// Creates a transformation matrix that represents shearing by the specified factor.
  /// </summary>
  /// <param name="factor">The shear factor.</param>
  /// <returns>A transformation matrix representing the shearing.</returns>
  public static TransformationMatrix Shear(double factor) {
    return new(1, factor, 0, 0, 1, 0);
  }

  #endregion Create

  #region Public Methods

  /// <summary>
  /// Attempts to decompose the transformation matrix into its constituent transformations.
  /// </summary>
  /// <param name="translation">The translation component.</param>
  /// <param name="rotation">The rotation component.</param>
  /// <param name="shear">The shear component.</param>
  /// <param name="scale">The scale component.</param>
  /// <returns>True if the decomposition is successful; otherwise, false.</returns>
  public bool TryDecompose(out TransformationMatrix translation,
                           out TransformationMatrix rotation,
                           out TransformationMatrix shear,
                           out TransformationMatrix scale) {
    var det = Determinant;

    if (det == 0 || !double.IsFinite(det)) {
      translation = Error;
      rotation = Error;
      shear = Error;
      scale = Error;

      return false;
    }

    var sx = Math.Sqrt(A11 * A11 + A22 * A22);
    var theta = Math.Atan2(A21, A11);
    var sy = det / sx;
    var k = (A11 * A12 + A21 * A22) / det;

    translation = Translate(A13, A23);
    rotation = Rotate(theta);
    shear = Shear(k);
    scale = Scale(sx, sy);

    return true;
  }

  /// <summary>
  /// Inverses the transformation matrix if it is invertible. If the determinant is zero, returns the Error matrix.
  /// </summary>
  /// <returns>The inverse transformation matrix, or the Error matrix if the determinant is zero.</returns>
  public TransformationMatrix Inverse() {
    double det = Determinant;

    if (det == 0) {
      return Error;
    }

    double invDet = 1.0 / det;

    double a11 = A22 * invDet;
    double a12 = -A12 * invDet;
    double a21 = -A21 * invDet;
    double a22 = A11 * invDet;

    double a13 = (A12 * A23 - A22 * A13) * invDet;
    double a23 = (A21 * A13 - A11 * A23) * invDet;

    return new TransformationMatrix(a11, a12, a13, a21, a22, a23);
  }

  /// <summary>
  /// Returns a string representation of the transformation matrix.
  /// </summary>
  /// <returns>String representation of the transformation matrix.</returns>
  public override string ToString() {
    return $"[{A11}, {A12}, {A13}; {A21}, {A22}, {A23}]";
  }

  #endregion Public Methods

  #region Operators

  /// <summary>
  /// Multiplies two transformation matrices, resulting in a new transformation matrix that combines the effects of both.
  /// </summary>
  /// <param name="left">The left transformation matrix.</param>
  /// <param name="right">The right transformation matrix.</param>
  /// <returns>The product of the two transformation matrices.</returns>
  public static TransformationMatrix operator *(TransformationMatrix left, TransformationMatrix right) {
    var a11 = left.A11 * right.A11 + left.A12 * right.A21;
    var a12 = left.A11 * right.A12 + left.A12 * right.A22;
    var a13 = left.A11 * right.A13 + left.A12 * right.A23 + left.A13;
    var a21 = left.A21 * right.A11 + left.A22 * right.A21;
    var a22 = left.A21 * right.A12 + left.A22 * right.A22;
    var a23 = left.A21 * right.A13 + left.A22 * right.A23 + left.A23;

    return new TransformationMatrix(a11, a12, a13, a21, a22, a23);
  }

  /// <summary>
  /// Multiplies a transformation matrix by a scalar value, resulting in a new transformation matrix where each element is scaled by the given scalar.
  /// </summary>
  /// <param name="matrix">The transformation matrix.</param>
  /// <param name="scalar">The scalar value.</param>
  /// <returns>The product of the transformation matrix and the scalar.</returns>
  public static TransformationMatrix operator *(TransformationMatrix matrix, double scalar) {
    return new TransformationMatrix(
      matrix.A11 * scalar,
      matrix.A12 * scalar,
      matrix.A13 * scalar,
      matrix.A21 * scalar,
      matrix.A22 * scalar,
      matrix.A23 * scalar
    );
  }

  /// <summary>
  /// Multiplies a scalar value by a transformation matrix, resulting in a new transformation matrix where each element is scaled by the given scalar.
  /// </summary>
  /// <param name="scalar">The scalar value.</param>
  /// <param name="matrix">The transformation matrix.</param>
  /// <returns>The product of the scalar and the transformation matrix.</returns>
  public static TransformationMatrix operator *(double scalar, TransformationMatrix matrix) {
    return matrix * scalar;
  }

  /// <summary>
  /// Divides a transformation matrix by a scalar value, resulting in a new transformation matrix where each element is divided by the given scalar. If the scalar is zero, returns the Error matrix.
  /// </summary>
  /// <param name="matrix">The transformation matrix.</param>
  /// <param name="scalar">The scalar value.</param>
  /// <returns>The quotient of the transformation matrix and the scalar.</returns>
  public static TransformationMatrix operator /(TransformationMatrix matrix, double scalar) {
    if (scalar == 0) {
      return Error;
    }

    return new TransformationMatrix(
      matrix.A11 / scalar,
      matrix.A12 / scalar,
      matrix.A13 / scalar,
      matrix.A21 / scalar,
      matrix.A22 / scalar,
      matrix.A23 / scalar
    );
  }

  /// <summary>
  /// Divides a scalar value by a transformation matrix, resulting in a new transformation matrix that is the inverse of the original matrix scaled by the scalar. If the determinant of the matrix is zero, returns the Error matrix.
  /// </summary>
  /// <param name="scalar">The scalar value.</param>
  /// <param name="matrix">The transformation matrix.</param>
  /// <returns>The quotient of the scalar and the transformation matrix.</returns>
  public static TransformationMatrix operator /(double scalar, TransformationMatrix matrix) {
    if (matrix.Determinant == 0) {
      return Error;
    }

    double invDet = scalar / matrix.Determinant;

    double a11 = matrix.A22 * invDet;
    double a12 = -matrix.A12 * invDet;
    double a21 = -matrix.A21 * invDet;
    double a22 = matrix.A11 * invDet;

    double a13 = (matrix.A12 * matrix.A23 - matrix.A22 * matrix.A13) * invDet;
    double a23 = (matrix.A21 * matrix.A13 - matrix.A11 * matrix.A23) * invDet;

    return new TransformationMatrix(a11, a12, a13, a21, a22, a23);
  }

  /// <summary>
  /// Divides one transformation matrix by another, resulting in a new transformation matrix that represents the combined effect of the first matrix followed by the inverse of the second matrix. If the determinant of the second matrix is zero, returns the Error matrix.
  /// </summary>
  /// <param name="left">The left transformation matrix.</param>
  /// <param name="right">The right transformation matrix.</param>
  /// <returns>The quotient of the two transformation matrices.</returns>
  public static TransformationMatrix operator /(TransformationMatrix left, TransformationMatrix right) {
    return left * right.Inverse();
  }

  /// <summary>
  /// Determines whether two <see cref="TransformationMatrix"/> instances are equal.
  /// </summary>
  /// <param name="left">The left transformation matrix.</param>
  /// <param name="right">The right transformation matrix.</param>
  /// <returns>True if the two transformation matrices are equal; otherwise, false.</returns>
  public static bool operator ==(TransformationMatrix left, TransformationMatrix right) {
    return left.Equals(right);
  }

  /// <summary>
  /// Determines whether two <see cref="TransformationMatrix"/> instances are not equal.
  /// </summary>
  /// <param name="left">The left transformation matrix.</param>
  /// <param name="right">The right transformation matrix.</param>
  /// <returns>True if the two transformation matrices are not equal; otherwise, false.</returns>
  public static bool operator !=(TransformationMatrix left, TransformationMatrix right) {
    return !left.Equals(right);
  }

  #endregion Operators

  #region IEquatable<TransformationMatrix>

  /// <summary>
  /// Determines whether the specified <see cref="TransformationMatrix"/> is equal to the current <see cref="TransformationMatrix"/>
  /// </summary>
  /// <param name="other">The <see cref="TransformationMatrix"/> to compare with the current <see cref="TransformationMatrix"/>.</param>
  /// <returns>True if the specified <see cref="TransformationMatrix"/> is equal to the current <see cref="TransformationMatrix"/>; otherwise, false.</returns>
  public bool Equals(TransformationMatrix other) {
    return A11 == other.A11 && A12 == other.A12 && A13 == other.A13 &&
           A21 == other.A21 && A22 == other.A22 && A23 == other.A23;
  }

  /// <summary>
  /// Determines whether the specified object is equal to the current <see cref="TransformationMatrix"/>.
  /// </summary>
  /// <param name="obj">The object to compare with the current <see cref="TransformationMatrix"/>.</param>
  /// <returns>True if the specified object is equal to the current <see cref="TransformationMatrix"/>; otherwise, false.</returns>
  public override bool Equals([NotNullWhen(true)] object? obj) =>
    obj is TransformationMatrix other && Equals(other);

  /// <summary>
  /// Returns a hash code for the current <see cref="TransformationMatrix"/>.
  /// </summary>
  /// <returns>Hash code for the current <see cref="TransformationMatrix"/>.</returns>
  public override int GetHashCode() => HashCode.Combine(A11, A12, A13, A21, A22, A23);

  #endregion IEquatable<TransformationMatrix>
}
