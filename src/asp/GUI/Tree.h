// __BEGIN_LICENSE__
//  Copyright (c) 2006-2013, United States Government as represented by the
//  Administrator of the National Aeronautics and Space Administration. All
//  rights reserved.
//
//  The NGT platform is licensed under the Apache License, Version 2.0 (the
//  "License"); you may not use this file except in compliance with the
//  License. You may obtain a copy of the License at
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
// __END_LICENSE__


#ifndef __STEREO_GUI_TREE_H__
#define __STEREO_GUI_TREE_H__

#include <vw/Core/Log.h>
#include <vw/Core/Exception.h>
#include <vw/Core/FundamentalTypes.h>

#include <boost/shared_array.hpp>
#include <map>

namespace vw {
namespace gui {

  /// TileNotFound exception
  ///
  /// This exception is thrown by the Tree and Index classes whenever
  /// a tile is requested that does not exist.  It is frequently
  /// caught by higher level classes like PlateFile when they are
  /// trying to determine whether a tile exists or not.
  ///
  VW_DEFINE_EXCEPTION(TileNotFoundErr, Exception);

  /// A subclass of TreeMapFunc can be used to iterate over a tree
  /// using the TreeNode::map() function below.
  struct TreeMapFunc {
    virtual ~TreeMapFunc() {}
    virtual void operator()(int32 col, int32 row, int32 level) = 0;
  };

  /// ???
  template <class ElementT>
  class TreeNode {
  
    typedef std::map<vw::int32,ElementT,std::greater<vw::int32> > record_type;

    TreeNode<ElementT> *m_parent;
    std::vector<boost::shared_ptr<TreeNode<ElementT> > > m_children;
    record_type m_records;
    int         m_num_levels;

    // ------------------------ Private Methods -----------------------------

    /// Do some quick arithmetic here to determine which child to follow
    /// in the quad tree for a given col, row, level request.
    /// - Returns 0, 1, 2, or 3.
    int compute_child_id(int col, int row, int level, int current_level);

    // Sets the child of this node with the 'id' according to the above index scheme.
    void set_child(int id, boost::shared_ptr<TreeNode> node) {
      m_children[id] = node;
    }

    // Insert the child of this node, but preserves the previous child's descendents.
    void insert_child(int id, boost::shared_ptr<TreeNode> node);


    // Search for a node at a given col, row, and level.
    ElementT search_helper(int col, int row, int level, int transaction_id, bool exact_transaction_match, int current_level);

    // Recursively call a function with valid [col, row, level] entries.
    //
    //    |---|---|
    //    | 0 | 1 |
    //    |---+---|
    //    | 2 | 3 |
    //    |---|---|
    //
    void map_helper(boost::shared_ptr<TreeMapFunc> func, int col, int row, int level);

    void insert_helper(ElementT const& record,
                       int col, int row, int level,
                       int transaction_id, int current_level,
                       bool insert_at_all_levels);

    void print_helper(int current_level) const;

  public:

    // ------------------------ Public Methods -----------------------------

    /// Use this constructor for the root of the tree....
    TreeNode() : m_parent(0), m_num_levels(0) {
      //      m_records[0] = ElementT();
      m_children.resize(4);
    }

    /// ... or use this contructor for the root of the tree.
    TreeNode(ElementT const& record, int transaction_id) : m_parent(0), m_num_levels(0) {
      m_records[transaction_id] = record;
      m_children.resize(4);
    }

    /// Use this contstructor to add record data immediately.
    TreeNode(TreeNode *parent, ElementT const& record, int transaction_id) :
      m_parent(parent), m_num_levels(0) {
      m_records[transaction_id] = record;
      m_children.resize(4);
    }

    // Return the child of this node with the 'id' according to the
    // following index scheme:
    //
    //    |---|---|
    //    | 0 | 1 |
    //    |---+---|
    //    | 2 | 3 |
    //    |---|---|
    //
    boost::shared_ptr<TreeNode> child(int id) const { return m_children[id]; }

    int num_children() const;

    /// ???
    ElementT value_helper(int transaction_id, bool exact_transaction_match);

    /// Access the data member of this node.
    ElementT value(int transaction_id, bool exact_transaction_match) {
      return value_helper(transaction_id, exact_transaction_match);
    }

    const ElementT value(int transaction_id, bool exact_transaction_match) const {
      return value_helper(transaction_id, exact_transaction_match);
    }

    /// Query max level of tree.
    int num_levels() const { return m_num_levels; }


    // Search for a node at a given col, row, and level.  Note: a
    // transaction ID of -1 indicates that we should return the
    // most recent tile, regardless of its transaction id.
    ElementT search(int col, int row, int level, int transaction_id, bool exact_transaction_match) {
      return search_helper(col, row, level, transaction_id, exact_transaction_match, 0);
    }

    // Insert an ElementT at a given position.  Intermediate nodes
    // in the tree are created (with empty ElementTs) in the tree
    // along the way, as needed.
    void insert(ElementT const& record, int col, int row,
                int level, int transaction_id,
                bool insert_at_all_levels = false);

    /// Print the tree.  (Use only for debugging small trees....)
    void print() const {
      vw_out() << "[ 0 -- Child 0 ] - ";
      this->print_helper(0);
    }

    void map(boost::shared_ptr<TreeMapFunc> func) {
      this->map_helper(func, 0, 0, 0);
    }

  };



}} // namespace vw

#include<Tree.tcc>


#endif // __VW_PLATE_TREE_H__
