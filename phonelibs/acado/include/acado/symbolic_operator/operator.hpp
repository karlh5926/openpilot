/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



/**
 *    \file include/acado/symbolic_operator/operator.hpp
 *    \author Boris Houska, Hans Joachim Ferreau
 */


#ifndef ACADO_TOOLKIT_OPERATOR_HPP
#define ACADO_TOOLKIT_OPERATOR_HPP


#include <acado/symbolic_operator/symbolic_operator_fwd.hpp>


BEGIN_NAMESPACE_ACADO


class Expression;
class ConstraintComponent;
class EvaluationBase;


/**
 *	\brief Abstract base class for all scalar-valued symbolic operators.
 *
 *	\ingroup BasicDataStructures
 *
 *	The class Operator serves as an abstract base class for all scalar-valued
 *	symbolic operators.
 *
 *	\author Boris Houska, Hans Joachim Ferreau
 */

class Operator{

public:


           /** Default constructor. */
           Operator();

    virtual ~Operator();


    /** Sets the argument (note that arg should have dimension 1). */

    virtual Operator& operator=( const double      & arg );
    virtual Operator& operator=( const DVector      & arg );
    virtual Operator& operator=( const DMatrix      & arg );
    virtual Operator& operator=( const Expression  & arg );
    virtual Operator& operator=( const Operator    & arg );


    Operator& operator+=( const double      & arg );
    Operator& operator+=( const DVector      & arg );
    Operator& operator+=( const DMatrix      & arg );
    Operator& operator+=( const Expression  & arg );

    Operator& operator-=( const double      & arg );
    Operator& operator-=( const DVector      & arg );
    Operator& operator-=( const DMatrix      & arg );
    Operator& operator-=( const Expression  & arg );

    Operator& operator*=( const double      & arg );
    Operator& operator*=( const DVector      & arg );
    Operator& operator*=( const DMatrix      & arg );
    Operator& operator*=( const Expression  & arg );

    Operator& operator/=( const double      & arg );
    Operator& operator/=( const Expression  & arg );


    Expression operator+( const double        & arg ) const;
    Expression operator+( const DVector        & arg ) const;
    Expression operator+( const DMatrix        & arg ) const;
    Expression operator+( const Operator& arg ) const;
    Expression operator+( const Expression    & arg ) const;

    friend Expression operator+( const double & arg1, const Operator& arg2 );
    friend Expression operator+( const DVector & arg1, const Operator& arg2 );
    friend Expression operator+( const DMatrix & arg1, const Operator& arg2 );

    Expression operator-( const double          & arg ) const;
    Expression operator-( const DVector          & arg ) const;
    Expression operator-( const DMatrix          & arg ) const;
    Expression operator-( const Operator  & arg ) const;
    Expression operator-( const Expression      & arg ) const;

    Expression operator-( ) const;

    friend Expression operator-( const double & arg1, const Operator& arg2 );
    friend Expression operator-( const DVector & arg1, const Operator& arg2 );
    friend Expression operator-( const DMatrix & arg1, const Operator& arg2 );

    Expression operator*( const double         & arg ) const;
    Expression operator*( const DVector         & arg ) const;
    Expression operator*( const DMatrix         & arg ) const;
    Expression operator*( const Operator & arg ) const;
    Expression operator*( const Expression     & arg ) const;

    friend Expression operator*( const double& arg1, const Operator& arg2 );
    friend Expression operator*( const DVector& arg1, const Operator& arg2 );
    friend Expression operator*( const DMatrix& arg1, const Operator& arg2 );

    Expression operator/( const double         & arg ) const;
    Expression operator/( const Operator & arg ) const;
    Expression operator/( const Expression     & arg ) const;


    friend Expression operator/( const double& arg1, const Operator& arg2 );
    friend Expression operator/( const DVector& arg1, const Operator& arg2 );
    friend Expression operator/( const DMatrix& arg1, const Operator& arg2 );

    ConstraintComponent operator<=( const double& ub ) const;
    ConstraintComponent operator>=( const double& lb ) const;
    ConstraintComponent operator==( const double&  b ) const;

    ConstraintComponent operator<=( const DVector& ub ) const;
    ConstraintComponent operator>=( const DVector& lb ) const;
    ConstraintComponent operator==( const DVector&  b ) const;

    ConstraintComponent operator<=( const VariablesGrid& ub ) const;
    ConstraintComponent operator>=( const VariablesGrid& lb ) const;
    ConstraintComponent operator==( const VariablesGrid&  b ) const;

    friend ConstraintComponent operator<=( double lb, const Operator &arg );
    friend ConstraintComponent operator==( double  b, const Operator &arg );
    friend ConstraintComponent operator>=( double ub, const Operator &arg );

    friend ConstraintComponent operator<=( DVector lb, const Operator &arg );
    friend ConstraintComponent operator==( DVector  b, const Operator &arg );
    friend ConstraintComponent operator>=( DVector ub, const Operator &arg );

    friend ConstraintComponent operator<=( VariablesGrid lb, const Operator &arg );
    friend ConstraintComponent operator==( VariablesGrid  b, const Operator &arg );
    friend ConstraintComponent operator>=( VariablesGrid ub, const Operator &arg );


    /** Evaluates the expression and stores the intermediate      \n
     *  results in a buffer (needed for automatic differentiation \n
     *  in backward mode)                                         \n
     *  \return SUCCESFUL_RETURN                   \n
     *          RET_NAN                            \n
     * */
    virtual returnValue evaluate( int     number    /**< storage position     */,
                                  double *x         /**< the input variable x */,
                                  double *result    /**< the result           */  ) = 0;


    /** Evaluates the expression (templated version) */
    virtual returnValue evaluate( EvaluationBase *x ) = 0;
	
	
    /** Returns the derivative of the expression with respect     \n
     *  to the variable var(index).                               \n
     *  \return The expression for the derivative.                \n
     *
     */
     virtual Operator* differentiate( int index     /**< diff. index    */ ) = 0;



    /** Automatic Differentiation in forward mode on the symbolic \n
     *  level. This function generates an expression for a        \n
     *  forward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                 \n
     */
     virtual Operator* AD_forward( int                dim      , /**< dimension of the seed */
                                     VariableType      *varType  , /**< the variable types    */
                                     int               *component, /**< and their components  */
                                     Operator       **seed     , /**< the forward seed      */
                                     int                &nNewIS  , /**< the number of new IS  */
                                     TreeProjection ***newIS    /**< the new IS-pointer    */ ) = 0;



    /** Automatic Differentiation in backward mode on the symbolic \n
     *  level. This function generates an expression for a         \n
     *  backward derivative                                        \n
     *  \return SUCCESSFUL_RETURN                                  \n
     */
    virtual returnValue AD_backward( int           dim      , /**< number of directions  */
                                     VariableType *varType  , /**< the variable types    */
                                     int          *component, /**< and their components  */
                                     Operator     *seed     , /**< the backward seed     */
                                     Operator    **df       , /**< the result            */
                                     int           &nNewIS  , /**< the number of new IS  */
                                     TreeProjection ***newIS  /**< the new IS-pointer    */ ) = 0;

    
    
    /** Automatic Differentiation in symmetric mode on the symbolic \n
     *  level. This function generates an expression for a          \n
     *  second order derivative.                                    \n
     *  \return SUCCESSFUL_RETURN                                   \n
     */
     virtual returnValue AD_symmetric( int            dim       , /**< number of directions  */
                                      VariableType  *varType   , /**< the variable types    */
                                      int           *component , /**< and their components  */
                                      Operator      *l         , /**< the backward seed     */
                                      Operator     **S         , /**< forward seed matrix   */
                                      int            dimS      , /**< dimension of forward seed             */
                                      Operator     **dfS       , /**< first order foward result             */
                                      Operator     **ldf       , /**< first order backward result           */
                                      Operator     **H         , /**< upper trianglular part of the Hessian */
                                      int            &nNewLIS  , /**< the number of newLIS  */
                                      TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                      int            &nNewSIS  , /**< the number of newSIS  */
                                      TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                      int            &nNewHIS  , /**< the number of newHIS  */
                                      TreeProjection ***newHIS   /**< the new HIS-pointer   */ ) = 0;



    /** Substitutes var(index) with the expression sub.           \n
     *  \return The substituted expression.                       \n
     *
     */
     virtual Operator* substitute( int   index            /**< subst. index    */,
                                     const Operator *sub  /**< the substitution*/) = 0;



    /** Checks whether the expression is zero or one              \n
     *  \return NE_ZERO                                           \n
     *          NE_ONE                                            \n
     *          NE_NEITHER_ONE_NOR_ZERO                           \n
     *
     */
     virtual NeutralElement isOneOrZero() const = 0;



     /** Asks the expression whether it is depending on a certian type of \n
       * variable.                                                        \n
       * \return BT_TRUE if a dependency is detected,                     \n
       *         BT_FALSE otherwise.                                      \n
       */
     virtual BooleanType isDependingOn( VariableType var ) const = 0;



    /** Checks whether the expression is depending on a variable  \n
     *  \return BT_FALSE if no dependence is detected             \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isDependingOn( int           dim      ,    /**< number of directions  */
                                          VariableType *varType  ,    /**< the variable types    */
                                          int          *component,    /**< and their components  */
                                          BooleanType   *implicit_dep /**< implicit dependencies */ ) = 0;




    /** Checks whether the expression is linear in                \n
     *  (or not depending on) a variable                          \n
     *  \return BT_FALSE if no linearity is                       \n
     *                detected                                    \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isLinearIn( int           dim      ,    /**< number of directions  */
                                       VariableType *varType  ,    /**< the variable types    */
                                       int          *component,    /**< and their components  */
                                       BooleanType  *implicit_dep  /**< implicit dependencies */ ) = 0;



    /** Checks whether the expression is polynomial in            \n
     *  the specified variables                                   \n
     *  \return BT_FALSE if the expression is not  polynomial     \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isPolynomialIn( int           dim      ,    /**< number of directions  */
                                           VariableType *varType  ,    /**< the variable types    */
                                           int          *component,    /**< and their components  */
                                           BooleanType  *implicit_dep  /**< implicit dependencies */ ) = 0;



    /** Checks whether the expression is rational in              \n
     *  the specified variables                                   \n
     *  \return BT_FALSE if the expression is not rational        \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isRationalIn( int           dim      ,    /**< number of directions  */
                                         VariableType *varType  ,    /**< the variable types    */
                                         int          *component,    /**< and their components  */
                                         BooleanType  *implicit_dep  /**< implicit dependencies */ ) = 0;

    /** Checks whether the expression is smooth in time           \n
     *  \return BT_FALSE if the expression is not smooth          \n
     *          BT_TRUE  otherwise                                \n
     *
     */
     virtual BooleanType isSmooth( ) const = 0;


    /** Returns the monotonicity of the expression.               \n
     *  \return MT_NONDECREASING                                  \n
     *          MT_NONINCREASING                                  \n
     *          MT_NONMONOTONIC                                   \n
     *
     */
     virtual MonotonicityType getMonotonicity( ) = 0;



    /** Returns the curvature of the expression                   \n
     *  \return CT_CONSTANT                                       \n
     *          CT_AFFINE                                         \n
     *          CT_CONVEX                                         \n
     *          CT_CONCAVE                                        \n
     *
     */
     virtual CurvatureType getCurvature( ) = 0;



    /** Overwrites the monotonicity of the expression.            \n
     *  (For the case that the monotonicity is explicitly known)  \n
     *  \return SUCCESSFUL_RETURN                                 \n
     *
     */
     virtual returnValue setMonotonicity( MonotonicityType monotonicity_ ) = 0;




    /** Overwrites the curvature of the expression.               \n
     *  (For the case that the curvature is explicitly known)     \n
     *  \return SUCCESSFUL_RETURN                                 \n
     *
     */
     virtual returnValue setCurvature( CurvatureType curvature_  ) = 0;



    /** Automatic Differentiation in forward mode.                \n
     *  This function stores the intermediate                     \n
     *  results in a buffer (needed for 2nd order automatic       \n
     *  differentiation in backward mode)                         \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward( int     number  /**< storage position */,
                                     double *x       /**< The evaluation
                                                          point x          */,
                                     double *seed    /**< the seed         */,
                                     double *f       /**< the value of the
                                                          expression at x  */,
                                     double *df      /**< the derivative of
                                                          the expression   */  ) = 0;



    /** Automatic Differentiation in forward mode.                \n
     *  This function uses the intermediate                       \n
     *  results from a buffer                                     \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward( int     number  /**< storage position */,
                                     double *seed    /**< the seed         */,
                                     double *df      /**< the derivative of
                                                          the expression   */  ) = 0;



    // IMPORTANT REMARK FOR AD_BACKWARD: run evaluate first to define
    //                                   the point x and to compute f.

    /** Automatic Differentiation in backward mode based on       \n
     *  buffered values                                           \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_backward( int    number /**< the buffer
                                                         position         */,
                                      double seed   /**< the seed         */,
                                      double  *df   /**< the derivative of
                                                         the expression   */) = 0;



    /** Automatic Differentiation in forward mode for             \n
     *  2nd derivatives.                                          \n
     *  This function uses intermediate                           \n
     *  results from a buffer.                                    \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_forward2( int    number  /**< the buffer
                                                          position         */,
                                      double *seed1  /**< the seed         */,
                                      double *seed2  /**< the seed for the
                                                          first derivative */,
                                      double *df     /**< the derivative of
                                                          the expression   */,
                                      double *ddf    /**< the 2nd derivative
                                                          of the expression*/) = 0;



    // IMPORTANT REMARK FOR AD_BACKWARD2: run AD_forward first to define
    //                                    the point x and to compute f and df.

    /** Automatic Differentiation in backward mode for 2nd order  \n
     *  derivatives based on buffered values.                     \n
     *  \return SUCCESFUL_RETURN                                  \n
     *          RET_NAN                                           \n
     */
     virtual returnValue AD_backward2( int    number /**< the buffer
                                                          position           */,
                                       double seed1  /**< the seed1          */,
                                       double seed2  /**< the seed2          */,
                                       double   *df  /**< the 1st derivative
                                                          of the expression  */,
                                       double  *ddf  /**< the 2nd derivative
                                                          of the expression  */   ) = 0;

     /** Prints the expression into a stream. \n
      *  \return SUCCESFUL_RETURN             \n
      */
     virtual std::ostream& print(std::ostream& stream) const = 0;


     /** Prints the expression into a stream ("flush" version). \n
      *  \return SUCCESFUL_RETURN                               \n
      */
     friend std::ostream& operator<<(std::ostream& stream, const Operator& arg);

     /** Provides a deep copy of the expression. \n
      *  \return a clone of the expression.      \n
      */
     virtual Operator* clone() const = 0;


     /** Provides a deep copy of a tree projection. \n
      *  \return a clone of the TreeProjection or   \n
      *          an assertion if the type this is   \n
      *          expression is no TreeProjection.   \n
      */
     virtual TreeProjection* cloneTreeProjection() const;



     /** Clears the buffer and resets the buffer size \n
      *  to 1.                                        \n
      *  \return SUCCESFUL_RETURN                     \n
      */
     virtual returnValue clearBuffer() = 0;



     /** Enumerates all variables based on a common   \n
      *  IndexList.                                   \n
      *  \return SUCCESFUL_RETURN
      */
     virtual returnValue enumerateVariables( SymbolicIndexList *indexList ) = 0;



     /** Asks the expression for its name.   \n
      *  \return the name of the expression. \n
      */
     virtual OperatorName getName() = 0;


     /** Asks the variable for its relative index. \n
      */
     //virtual int getVariableIndex( ) const;


     /** Asks the variable for its global index. \n
      */
     virtual int getGlobalIndex( ) const;


     /** Asks the expression whether it is a variable.   \n
      *  \return The answer. \n
      */
     virtual BooleanType isVariable( VariableType &varType,
                                     int &component          ) const = 0;


     /** The function loadIndices passes an IndexList through    \n
      *  the whole expression tree. Whenever a variable gets the \n
      *  IndexList it tries to make an entry. However if a       \n
      *  variable recognices that it has already been added      \n
      *  before it will not be allowed to make a second entry.   \n
      *  Note that all variables, in paticular the intermediate  \n
      *  states, will keep in mind whether they were allowed     \n
      *  to make an entry or not. This guarantees that           \n
      *  intermediate states are never evaluated twice if they   \n
      *  occur at several knots of the tree.                     \n
      *                                                          \n
      *  THIS FUNCTION IS FOR INTERNAL USE ONLY.                 \n
      *                                                          \n
      *  PLEASE CALL THIS FUNTION AT MOST ONES FOR AN EXPRESSION \n
      *  AS A KIND OF INIT ROUTINE.                              \n
      *                                                          \n
      *  \return the name of the expression.                     \n
      */
     virtual returnValue loadIndices( SymbolicIndexList *indexList /**< The index list to be
                                                                     *  filled with entries  */ ) = 0;



     /** Return the value of the constant */
     virtual double getValue() const;



     /** Returns the argument or NULL if no intermediate argument available */
     virtual Operator* passArgument() const;


    /** Asks whether all elements are purely symbolic.                \n
      *                                                               \n
      * \return BT_TRUE  if the complete tree is symbolic.            \n
      *         BT_FALSE otherwise (e.g. if C functions are linked).  \n
      */
    virtual BooleanType isSymbolic() const = 0;


    int nCount;



	/** Sets the name of the variable that is used for code export.   \n
	 *  \return SUCCESSFUL_RETURN                                     \n
	 */
    virtual returnValue setVariableExportName(	const VariableType &_type,
    											const std::vector< std::string >& _name
    											);


    
    
    virtual Operator* myProd(Operator* a,Operator* b);
    virtual Operator* myAdd (Operator* a,Operator* b);
    virtual Operator* mySubtract (Operator* a,Operator* b);
    virtual Operator* myPower (Operator* a,Operator* b);
    virtual Operator* myPowerInt (Operator* a,int b);
    virtual Operator* myLogarithm (Operator* a);


    virtual BooleanType isTrivial() const;


    virtual returnValue initDerivative();
    
    
//
//  PROTECTED FUNCTIONS:
//

protected:


    virtual TreeProjection* convert2TreeProjection( Operator* a ) const; // Caution: a is deleted inside...
    
    returnValue ADsymCommon( 	Operator     *a  ,
                              	TreeProjection &da ,
                              	TreeProjection &dda,
                                int            dim       , /**< number of directions  */
                                VariableType  *varType   , /**< the variable types    */
                                int           *component , /**< and their components  */
                                Operator      *l         , /**< the backward seed     */
                                Operator     **S         , /**< forward seed matrix   */
                                int            dimS      , /**< dimension of forward seed             */
                                Operator     **dfS       , /**< first order foward result             */
                                Operator     **ldf       , /**< first order backward result           */
                                Operator     **H         , /**< upper trianglular part of the Hessian */
                                int            &nNewLIS  , /**< the number of newLIS  */
                                TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                int            &nNewSIS  , /**< the number of newSIS  */
                                TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                int            &nNewHIS  , /**< the number of newHIS  */
                                TreeProjection ***newHIS   /**< the new HIS-pointer   */ );

	
    returnValue ADsymCommon2( 	  Operator       *a  ,
				   	   	   	   	  Operator       *b  ,
                                  TreeProjection &dx ,
                                  TreeProjection &dy ,
                                  TreeProjection &dxx,
                                  TreeProjection &dxy,
                                  TreeProjection &dyy,
                                  int            dim       , /**< number of directions  */
                                  VariableType  *varType   , /**< the variable types    */
                                  int           *component , /**< and their components  */
                                  Operator      *l         , /**< the backward seed     */
                                  Operator     **S         , /**< forward seed matrix   */
                                  int            dimS      , /**< dimension of forward seed             */
                                  Operator     **dfS       , /**< first order foward result             */
                                  Operator     **ldf       , /**< first order backward result           */
                                  Operator     **H         , /**< upper trianglular part of the Hessian */
                                  int            &nNewLIS  , /**< the number of newLIS  */
                                  TreeProjection ***newLIS , /**< the new LIS-pointer   */
                                  int            &nNewSIS  , /**< the number of newSIS  */
                                  TreeProjection ***newSIS , /**< the new SIS-pointer   */
                                  int            &nNewHIS  , /**< the number of newHIS  */
                                  TreeProjection ***newHIS   /**< the new HIS-pointer   */ );

    //
    //  PROTECTED MEMBERS:
    //

    protected:

      	BooleanType  initialized;


};


CLOSE_NAMESPACE_ACADO



#endif



