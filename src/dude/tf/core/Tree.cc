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

#include "Tree.h"
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphviz.hpp>

#include <algorithm>
#include <deque>
#include <fstream>
#include <iostream>
#include <list>

using namespace dude::tf;

Tree::Tree() {
	m_graph = Graph_t();
}

bool Tree::add_element(const id_t &unique_id, const Tree::FrameType_t &element,
                       const id_t &parent) {
	// check if node already exists
	if(exists(unique_id)) {
		return false;
	}
	if(parent != NO_ID and not exists(parent)) {
		return false;
	}

	// add new element
	auto vx_id     = boost::add_vertex(m_graph);
	m_graph[vx_id] = {.id = unique_id, .frame = element, .parent = parent};

	// check if child or root
	if(parent != NO_ID) {
		// Get parent and add edges
		const auto vx_parent      = get_vertex_by_id(parent);
		const auto to_child_edge  = boost::add_edge(vx_parent, vx_id, m_graph).first;
		const auto to_parent_edge = boost::add_edge(vx_id, vx_parent, m_graph).first;

		// Set edge properties
		m_graph[to_child_edge].is_forward  = true;
		m_graph[to_parent_edge].is_forward = false;
	}

	return true;
}

void Tree::clear() {
	m_graph.clear();
}

std::size_t Tree::size() const {
	return boost::num_vertices(m_graph);
}

Tree::FrameType_t &Tree::get(const id_t &id) {
	return m_graph[get_vertex_by_id(id)].frame;
}

Tree::Graph_t::vertex_descriptor Tree::get_vertex_by_id(const dude::tf::id_t &id) const {
	// get all vertices
	Graph_t::vertex_iterator it, end;
	std::tie(it, end) = boost::vertices(m_graph);

	// find vertex with correct id
	auto result = std::find_if(it, end, [id, this](const Graph_t::vertex_descriptor &vx) {
		return this->m_graph[vx].id == id;
	});

	return *result;
}

bool Tree::exists(const id_t &id) const {
	// get all vertices
	Graph_t::vertex_iterator it, end;
	std::tie(it, end) = boost::vertices(m_graph);

	// find vertex with correct id
	auto result = std::find_if(it, end, [id, this](const Graph_t::vertex_descriptor &vx) {
		return this->m_graph[vx].id == id;
	});

	return result != end;
}

bool Tree::move_element(const dude::tf::id_t &id, const dude::tf::id_t &new_parent) {
	if(not exists(id) or not exists(new_parent)) {
		// TODO output error
		return false;
	}

	// remove all edges to current parent
	auto vx_source = get_vertex_by_id(id);
	if(m_graph[vx_source].parent != NO_ID) {
		const auto &vx_current_parent = get_vertex_by_id(m_graph[vx_source].parent);

		// remove forward edge
		Graph_t::edge_descriptor edge;
		bool edge_exists;
		std::tie(edge, edge_exists) = boost::edge(vx_current_parent, vx_source, m_graph);
		if(edge_exists) {
			boost::remove_edge(edge, m_graph);
		}

		// remove reverse edge
		std::tie(edge, edge_exists) = boost::edge(vx_source, vx_current_parent, m_graph);
		if(edge_exists) {
			boost::remove_edge(edge, m_graph);
		}
	}

	// add new edges
	if(new_parent != NO_ID) {
		const auto &vx_new_parent = get_vertex_by_id(new_parent);

		// insert edges
		const auto edge_forward = boost::add_edge(vx_new_parent, vx_source, m_graph).first;
		const auto edge_reverse = boost::add_edge(vx_source, vx_new_parent, m_graph).first;

		// set properties
		m_graph[edge_forward].is_forward = true;
		m_graph[edge_reverse].is_forward = false;
	}
}

bool Tree::remove_element(const dude::tf::id_t &id) {
	if(not exists(id)) {
		// TODO output error
		return false;
	}

	// find vertex id
	const auto &vx_id = get_vertex_by_id(id);

	// remove all children and their edges using a
	// BFS approach
	std::deque<id_t> open_list, children_to_delete;

	// add given id to lists
	open_list.push_back(id);
	children_to_delete.push_back(id);

	while(not open_list.empty()) {
		// get next element and remove from queue
		const auto next_id = open_list.back();
		open_list.pop_back();

		// iterate over all children and mark them for removal
		for(const auto &child: get_all_children(next_id)) {
			open_list.push_back(child);
			children_to_delete.push_back(child);
		}
	}

	// remove all elements and their edges
	for(const auto &current_id: children_to_delete) {
		const auto &vx_current = get_vertex_by_id(current_id);

		// remove all edges from and to this vertex
		boost::clear_vertex(vx_current, m_graph);

		// finally, remove the vertex
		boost::remove_vertex(vx_current, m_graph);
	}

	return true;
}

std::vector<std::pair<dude::tf::id_t, Tree::FrameType_t>> Tree::get_all_elements() const {
	std::vector<std::pair<id_t, FrameType_t>> list;

	// iterate over all vertices and add them to the list
	Graph_t::vertex_iterator it, end;
	std::tie(it, end) = boost::vertices(m_graph);
	for(; it != end; ++it) {
		list.emplace_back(m_graph[*it].id, m_graph[*it].frame);
	}

	return list;
}

std::vector<Tree::PathItem> Tree::get_all_connections() const {
	std::vector<PathItem> list;

	// iterate over all edges and add them to the list
	Graph_t::edge_iterator it, end;
	std::tie(it, end) = boost::edges(m_graph);
	for(; it != end; ++it) {
		list.emplace_back(PathItem{.source = m_graph[boost::source(*it, m_graph)].id,
		                           .target = m_graph[boost::target(*it, m_graph)].id});
	}

	return list;
}

std::vector<Tree::PathItem> Tree::find_path(const dude::tf::id_t &source,
                                            const dude::tf::id_t &target) const {
	// check if source and target exist
	if(not exists(source) or not exists(target)) {
		// TODO output error
		return {};
	}

	// get source and target vertices
	auto vx_source = get_vertex_by_id(source);
	auto vx_target = get_vertex_by_id(target);

	// search for the path
	Tree::GraphVisitor::PredecessorMap_t map;
	map[vx_source] = {.predecessor = vx_source, .is_forward = false};
	Tree::GraphVisitor vis(&map);
	boost::breadth_first_search(m_graph, vx_source, boost::visitor(vis));

	// extract path
	Graph_t::vertex_descriptor current = vx_target;
	std::list<PathItem> path;
	while(map[current].predecessor != current) {
		// emplace IDs into path
		path.emplace_front(
				PathItem{.source = m_graph[map[current].predecessor].id, .target = m_graph[current].id});
		current = map[current].predecessor;
	}

	// check if we have a path
	if(current != vx_source) {
		return {};  // empty because no path was found
	}

	// return as vector
	return {path.begin(), path.end()};
}

std::vector<dude::tf::id_t> Tree::get_all_children(const dude::tf::id_t &id) const {
	std::vector<id_t> list;

	Graph_t::out_edge_iterator it, end;
	std::tie(it, end) = boost::out_edges(get_vertex_by_id(id), m_graph);
	for(; it != end; ++it) {
		if(m_graph[*it].is_forward) {
			const auto vx_child = boost::target(*it, m_graph);
			list.push_back(m_graph[vx_child].id);
		}
	}

	return list;
}
void Tree::write_graph(const std::string &output_path) const {
	// create stream for output file
	auto file = std::ofstream(output_path);

	// create writer object
	auto writer = GraphvizWriter(m_graph);

	// write
	boost::write_graphviz(file, m_graph, writer, writer);
}

void Tree::set(const dude::tf::id_t &id, const Tree::FrameType_t &frame) {
	get(id) = frame;
}

Tree::FrameType_t Tree::get(const Tree::PathItem &item) {
	// find out if reverse or not
	const auto ex =
			boost::edge(get_vertex_by_id(item.source), get_vertex_by_id(item.target), m_graph).first;

	if(m_graph[ex].is_forward) {
		// if forward, then return the normal Frame of the target
		return m_graph[get_vertex_by_id(item.target)].frame;
	} else {
		// otherwise, we need the inverse() of the source
		return m_graph[get_vertex_by_id(item.source)].frame.inverse();
	}
}

std::vector<dude::tf::id_t> Tree::get_all_roots() const {
	std::vector<id_t> roots;

	// get all vertices
	Graph_t::vertex_iterator it, end;
	std::tie(it, end) = boost::vertices(m_graph);

	for(; it != end; ++it) {
		Graph_t::out_edge_iterator edges_it, edges_end;
		std::tie(edges_it, edges_end) = boost::out_edges(*it, m_graph);

		// this is a root element if all outgoing edges are forward
		const auto result = std::all_of(
				edges_it, edges_end,
				[this](const Graph_t::edge_descriptor &ex) { return this->m_graph[ex].is_forward; });

		// add id of the element to list
		if(result) {
			roots.emplace_back(m_graph[*it].id);
		}
	}

	return roots;
}

std::optional<Tree::FrameType_t> Tree::get_composed_transform(const dude::tf::id_t &source,
                                                              const dude::tf::id_t &target) {
	auto path_elements = find_path(source, target);

	// return an empty element if no path found
	if(path_elements.empty()) {
		return {};
	}

	FrameType_t composed;
	for(const auto &element: path_elements) {
		// compose step-by-step
		const auto &element_frame = get(element);
		composed                  = composed.compose(element_frame);
	}

	return {composed};
}

dude::tf::id_t &Tree::get_parent_id(const dude::tf::id_t &id) {
	const auto &element_idx = get_vertex_by_id(id);
	return m_graph[element_idx].parent;
}
