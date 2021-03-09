// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    Edge.hpp
 * @author  Jared Strader
 */
//-----------------------------------------------------------------------------

#ifndef Edge_HPP
#define Edge_HPP

class Edge {
  public:

  /** \brief Initialize weighted edge pointing from src to dst */
  inline Edge(int src,
              int dst,
              double weight) 
  : src_(src),
    dst_(dst),
    weight_(weight) {};

  /** \brief Index of edge tail */
  int src_;

  /** \brief Index of edge head */
  int dst_;

  /** \brief Weight of edge */
  double weight_;

};

#endif // Edge_HPP
