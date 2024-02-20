// *********************************************************
// tf_dude
// (c) 2022-2024
// Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
// Institute fuer Robotik und Mechatronik
//
// German Aerospace Center
// Institute for Robotics and Mechatronics
//
// This file is part of the tf_dude repository and is provided as is.
// The repository's license does apply.
// 
// *********************************************************
//
// Authors:
// Sewtz, Marco
//
// *********************************************************

#ifndef TF_DUDE_TREE_H
#define TF_DUDE_TREE_H

#include "Frame.h"
#include "Types.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace dude::tf {

class Tree {
	struct EdgeData;
	struct VertexData;

public:
	// structs for strong typing
	struct PathItem;

	/* Graph typename */
	using Graph_t = boost::adjacency_list<boost::setS,       // std set
	                                      boost::vecS,       // std vector
	                                      boost::directedS,  // TODO
	                                      VertexData,        // vertex data struct
	                                      EdgeData           // edge data struct
	                                      >;

	/* Typename of the used treenode types */
	using FrameType_t = Frame;

	/**
	 * \brief Create an empty tree with no elements
	 */
	Tree();

	/**
	 * \brief Add a new element to the tree
	 * \param unique_id the unique id of the element
	 * \param element the new element to add
	 * \param parent the id of the parent element, otherwise will become root element
	 * \return True if successful, False otherwise
	 */
	bool add_element(const id_t& unique_id, const FrameType_t& element, const id_t& parent = NO_ID);

	/**
	 * \brief Move a given element to a new parent. Will
	 *        Also move all children
	 * \param id unique identifier of the element
	 * \param new_parent the id of the new parent element
	 * \return True if successful
	 */
	bool move_element(const id_t& id, const id_t& new_parent);

	/**
	 * \brief Remove an element from the tree
	 * \param id requested id to be removed
	 * \return True if successful, False otherwise
	 */
	bool remove_element(const id_t& id);

	/**
	 * \brief Find and get access to frame
	 * \param id unique id of parent
	 * \return reference to parent node
	 */
	FrameType_t& get(const id_t& id);

	/**
	 * \brief Get the correct frame for the given PathItem
	 * \param item PathItem as obtained form find_path()
	 * \return Frame. If the PathItem points to a reversed
	 *         path, this frame is already inverse.
	 */
	FrameType_t get(const PathItem& item);

	/**
	 * \brief Get the id of the parent element of the given
	 * element id.
	 * \param id id of the child element
	 * \return unique id of the parent. Will be empty if root
	 * element
	 */
	[[nodiscard]] id_t& get_parent_id(const id_t& id);

	/**
	 * \brief Update data of given frame
	 * \param id unique id
	 * \param frame new frame data
	 */
	void set(const id_t& id, const FrameType_t& frame);

	/**
	 * \brief Return whether a given id exists in
	 *        the tree structure
	 * \param id unique id
	 * \return True if found, False otherwise
	 */
	[[nodiscard]] bool exists(const id_t& id) const;

	/**
	 * \brief Clear all elements
	 */
	void clear();

	/**
	 * \brief tell whether the tree is empty
	 * \return True if empty, False otherwise
	 */
	[[nodiscard]] bool empty() const {
		return size() == 0;
	};

	/**
	 * \brief Return the number of elements in the tree
	 * \return Number of elements
	 */
	[[nodiscard]] std::size_t size() const;

	/**
	 * \brief Return a vector with all frames
	 * \return vector with a pair for each frame
	 *         first -> unique identifier
	 *         second -> frame
	 */
	[[nodiscard]] std::vector<std::pair<id_t, FrameType_t>> get_all_elements() const;

	/**
	 * \brief Return a vector with all connections
	 * \return vector with a pair for each connection
	 *         first -> parent identifier
	 *         second -> child identifier
	 */
	[[nodiscard]] std::vector<PathItem> get_all_connections() const;

	/**
	 * \brief Find the path from the source to the target. The returned path
	 *         will include the target and include the source. If no path has
	 *         been found, an empty vector will be returned
	 * \param source starting element
	 * \param target ending element
	 * \return Path vector, each element containing a pair
	 *         first -> element id_t
	 *         second -> bool if edge has been traversed forward
	 *                   or reversed
	 */
	[[nodiscard]] std::vector<PathItem> find_path(const id_t& source, const id_t& target) const;

	/**
	 * \brief Find and compose the transform along the path specified by source and target.
	 *        If no path could be found, the return value will be empty
	 * \param source starting element
	 * \param target ending element
	 * \return Composed transform from source to target, empty value if no path was found
	 */
	[[nodiscard]] std::optional<FrameType_t> get_composed_transform(const id_t& source,
	                                                                const id_t& target);

	/**
	 * \brief Return a list of all children of this element
	 * \param id the unique id of the requested element
	 * \return vector containing all unique identifiers
	 */
	[[nodiscard]] std::vector<id_t> get_all_children(const id_t& id) const;

	/**
	 * \brief Return a list of all top-level elements
	 * \return vector containing all unique identifiers
	 */
	[[nodiscard]] std::vector<id_t> get_all_roots() const;

	/**
	 * \brief Write the current graph as graphviz to a
	 *        specified file
	 * \param output_path path to file
	 */
	void write_graph(const std::string& output_path) const;

protected:
	/**
	 * \brief Get the vertex descriptor for the given element
	 *        identifier
	 * \param id unique identifier
	 * \return vertex descriptor
	 */
	[[nodiscard]] Graph_t::vertex_descriptor get_vertex_by_id(const id_t& id) const;

public:
	struct PathItem {
		id_t source;
		id_t target;
	};

private:
	/* Graph structure */
	Graph_t m_graph;

	/**
	 * \brief A struct holding info on the edge
	 */
	struct EdgeData {
		/* if True, the edge is from parent to child, False otherwise */
		bool is_forward;
	};

	/**
	 * \brief A struct holding info on the vertex
	 */
	struct VertexData {
		/* the unique identifier of this node */
		id_t id;

		/* the frame data */
		FrameType_t frame;

		/* id of parent */
		id_t parent = NO_ID;
	};

	struct PredecessorItem {
		Graph_t::vertex_descriptor predecessor;
		bool is_forward;
	};

	/**
	 * \brief A visitor implementation for tracking the path in the BFS
	 *        search
	 */
	struct GraphVisitor : boost::bfs_visitor<> {
		/* Typedef for Predecessor Map */
		using PredecessorMap_t = std::map<Graph_t::vertex_descriptor, PredecessorItem>;

		/**
		 * \brief Create a visitor and add a pointer to the predecessor
		 *        map
		 * \param p_map pointer to predecessor map
		 */
		explicit GraphVisitor(PredecessorMap_t* p_map) : mp_predecessor_map(p_map) {}

		/**
		 * \brief A function which is called everytime an edge is added
		 * \param e Edge descriptor
		 * \param g Graph descriptor
		 */
		template <class Edge, class Graph>
		void tree_edge(Edge e, Graph& g) {
			mp_predecessor_map->insert(std::make_pair(
					boost::target(e, g),
					PredecessorItem{.predecessor = boost::source(e, g), .is_forward = g[e].is_forward}));
		}

	private:
		PredecessorMap_t* mp_predecessor_map;
	};

	/**
	 * \brief
	 */
	class GraphvizWriter {
	public:
		explicit GraphvizWriter(const Graph_t& graph) : mp_graph(&graph) {}

		void operator()(std::ostream& out, const Graph_t::vertex_descriptor& vx) const {
			out << "[label=\"" << (*mp_graph)[vx].id << "\"]";
		}

		void operator()(std::ostream& out, const Graph_t::edge_descriptor& ex) const {
			if((*mp_graph)[ex].is_forward) {
				out << "[color=black]";
			} else {
				out << "[style=invis]";
			}
		}

	private:
		const Graph_t* mp_graph;
	};
};
}  // namespace dude::tf
#endif  // TF_DUDE_TREE_H
