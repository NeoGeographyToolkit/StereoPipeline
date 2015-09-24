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




namespace vw {
namespace gui {


template <class ElementT>
int TreeNode<ElementT>::compute_child_id(int col, int row, int level, int current_level) {

  int tile_x = col / (1 << (level-current_level));
  int tile_y = row / (1 << (level-current_level));

  if (tile_x >= (1 << current_level) ||
      tile_y >= (1 << current_level))
    vw_throw(TileNotFoundErr() << "TreeNode: invalid index ("
             << col << " " << row << " at level " << level << ").");
  tile_x %= 2;
  tile_y %= 2;

  // For debugging
  // std::cout << "Adding at " << tile_x << " " << tile_y << "   "
  //           << col << " " << row << "  " << level << " " << current_level << "\n";
  if (tile_x == 0 && tile_y == 0)
    return 0;
  else if (tile_x == 1 && tile_y == 0)
    return 1;
  else if (tile_x == 0 && tile_y == 1)
    return 2;
  else
    return 3;
}


template <class ElementT>
void TreeNode<ElementT>::insert_child(int id, boost::shared_ptr<TreeNode> node) {

  // First we save the old child and replace it with the new one.
  boost::shared_ptr<TreeNode> old_child = m_children[id];
  m_children[id] = node;

  // Then, if the old child existed, we transfer the old child's
  // children (our grandchildren) to the new child node.
  if (old_child)
    for (int i = 0; i < 4 ; ++i)
      m_children[id]->set_child(i, old_child->child(i));
}


template <class ElementT>
ElementT TreeNode<ElementT>::search_helper(int col, int row, int level, int transaction_id, bool exact_transaction_match, int current_level) {
  // For debugging:
  // std::cout << "Call to search_helper(" << col << " " << row << " " << level << " "
  //           << transaction_id << " " << current_level << ")\n";

  // If we have reached the requested level, then we must be at
  // the node we want!  Return the ElementT!
  if (current_level == level) {

    // Handle the edge case where the user has requested a tile
    // outside of the 1x1 bounds of the root level.
    if ( level == 0 && (col !=0 || row != 0) )
      vw_throw(TileNotFoundErr() << "TreeNode: invalid index (" << col << " " << row << ").");

    return this->value(transaction_id, exact_transaction_match);

  // Otherwise, we go recurse deeper into the tree....
  } else {

    int child_id = compute_child_id(col, row, level, current_level + 1);

    if (m_children[child_id]) {

      // If a branch of the tree is found, we dive deeper.
      return m_children[child_id]->search_helper(col, row, level,
                                                 transaction_id, exact_transaction_match,
                                                 current_level + 1);

    } else {
      // If not, we throw an exception.
      vw_throw(TileNotFoundErr() << "Tile search [" << col << " " << row << " "
               << current_level << "] failed at level " << current_level << "\n");
    }
  }
  // If not, we throw an exception.
  vw_throw(TileNotFoundErr() << "Tile search [" << col << " " << row << " "
           << current_level << "] failed at level " << current_level << "\n");
}

template <class ElementT>
void TreeNode<ElementT>::map_helper(boost::shared_ptr<TreeMapFunc> func, int col, int row, int level) {

  // Call the function for the current level.
  (*func)(col, row, level);

  // Call the function for future levels.
  if ( this->child(0) )
    this->child(0)->map_helper(func, col*2, row*2, level + 1);

  if ( this->child(1) )
    this->child(1)->map_helper(func, col*2+1, row*2, level + 1);

  if ( this->child(2) )
    this->child(2)->map_helper(func, col*2, row*2+1, level + 1);

  if ( this->child(3) )
    this->child(3)->map_helper(func, col*2+1, row*2+1, level + 1);
}

template <class ElementT>
void TreeNode<ElementT>::insert_helper(ElementT const& record,
                   int col, int row, int level,
                   int transaction_id, int current_level,
                   bool insert_at_all_levels) {

  // If we have reached the requested level, then we must be at
  // the node we want!  Return the ElementT!
  if (current_level == level) {

    // Handle the edge case where the user has requested a tile
    // outside of the 1x1 bounds of the root level.
    if ( level == 0 && (col !=0 || row != 0) )
      vw_throw(TileNotFoundErr() << "TreeNode: invalid index (" << col << " " << row << ").");
    // Add the record for the given transaction ID.
    m_records[transaction_id] = record;

  // Otherwise, we need to recurse further into the tree....
  } else {

    int child_id = this->compute_child_id(col, row, level, current_level+1);

    // If the child we need is not yet created, we create it, add
    // it as our child, and recurse down it.
    if (!m_children[child_id]) {
      boost::shared_ptr<TreeNode> node( new TreeNode(this, ElementT(), transaction_id ) );
      this->set_child(child_id, node);
    }

    // Insert the record at this level if requested.  Right now this is mostly a
    // special feature used by the transaction_request() code in
    // LocalIndex to "prime" the tree with empty index entries.
    if (insert_at_all_levels)
      m_records[transaction_id] = record;

    m_children[child_id]->insert_helper(record, col, row, level,
                                        transaction_id, current_level+1,
                                        insert_at_all_levels);
  }

}


template <class ElementT>
void TreeNode<ElementT>::print_helper(int current_level) const {
  vw_out() << (*(m_records.begin())).second.status() << "\n";
  for (int i = 0; i < 4; ++i)
    if ( this->child(i) ) {
      for (int l = 0; l < current_level+1; ++l)
        vw_out() << "  ";
      vw_out() << "[ " << (current_level+1)
                 << " -- Child " << i << " ] - " ;
      this->child(i)->print_helper(current_level + 1);
    }
}


template <class ElementT>
int TreeNode<ElementT>::num_children() const {
  int n = 0;
  for (int i = 0; i < 4; ++ i)
    if (m_children[i])
      ++n;
  return n;
}

template <class ElementT>
ElementT value_helper(int transaction_id, bool exact_transaction_match) {
  typename record_type::iterator it = m_records.begin();

  // A transaction ID of -1 indicates that we should return the
  // most recent tile, regardless of its transaction id.
  if (transaction_id == -1 && it != m_records.end())
    return (*it).second;

  // Otherwise, we search for the most recent record that happened
  // before on on the queried ephoch.
  while (it != m_records.end()) {
    if (exact_transaction_match) {
      if ((*it).first == transaction_id)
        return (*it).second;
    } else {
      if ((*it).first <= transaction_id)
        return (*it).second;
    }
    ++it;
  }

  // If we reach this point, then there are no entries before
  // the given transaction_id, so we return an empty (and invalid) record.
  vw_throw(TileNotFoundErr() << "Tiles exist at this location, but none before transaction_id="
           << transaction_id << "\n");
  return ElementT(); // never reached
}



template <class ElementT>
void TreeNode<ElementT>::insert(ElementT const& record, int col, int row,
            int level, int transaction_id,
            bool insert_at_all_levels = false) {
  this->insert_helper(record, col, row, level, transaction_id, 0, insert_at_all_levels);
  if (level >= m_num_levels)
    m_num_levels = level+1;
}



}} // namespace vw


