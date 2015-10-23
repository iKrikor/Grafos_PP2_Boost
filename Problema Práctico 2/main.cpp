//
//  main.cpp
//  Problema Práctico 2
//
//  Created by Krikor Bisdikian on 10/18/15.
//  Copyright © 2015 Krikor Bisdikian. All rights reserved.
//

#include <iostream>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <vector>
#include <chrono>

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/exterior_property.hpp>


using namespace boost;


struct vertex_info {
    std::string id;
};

typedef std::chrono::microseconds micro;

typedef property < edge_weight_t, float > Weight;
typedef adjacency_list < vecS, vecS, directedS, vertex_info, Weight > DirectedGraph;
typedef typename DirectedGraph::edge_property_type Weight;

typedef graph_traits<DirectedGraph>:: vertex_descriptor vertex_descriptor;
typedef property_map<DirectedGraph, edge_weight_t>::type WeightMap;

typedef exterior_vertex_property<DirectedGraph, float> DistanceProperty;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;




template < typename DirectedGraph >
void insertVertex(std::string id, DirectedGraph & digraph);
template < typename DirectedGraph >
void insertEdge(std::string vert1, std::string vert2, double w, DirectedGraph & digraph);
template < typename DirectedGraph >
void deleteVert(std::string vert1, DirectedGraph & digraph);
template < typename DirectedGraph >
void deleteEdge(std::string vert1, std::string vert2, DirectedGraph & digraph);
template < typename DirectedGraph >
void dfs(DirectedGraph & digraph);
template < typename DirectedGraph >
void bfs(DirectedGraph & digraph);
template < typename DirectedGraph >
void prim(DirectedGraph & digraph);
template < typename DirectedGraph >
void kruskal(DirectedGraph & digraph);
template < typename DirectedGraph >
void dijkstra(DirectedGraph & digraph, std::string id);
template < typename DirectedGraph >
void floydwarshal(DirectedGraph & digraph);
void ejecuta(DirectedGraph & digraph);


class custom_bfs_visitor : public boost::default_bfs_visitor
{
public:
    

    void discover_vertex(vertex_descriptor u, const DirectedGraph & digraph) const
    {
        std::cout << digraph[u].id << std::endl;
    }
};

class custom_dfs_visitor : public boost::default_dfs_visitor
{
public:
    
    void discover_vertex(vertex_descriptor u, const DirectedGraph & digraph) const
    {
        std::cout << digraph[u].id << std::endl;
    }
};

int main(int argc, const char * argv[]) {
    
    
    DirectedGraph digraph;
    
    int op;
    std::string vert1;
    std::string vert2;
    double w;
    
    std::cout<<"Elija una de las siguientes opciones\n 1. Insertar un vértice\n 2.Insertar arista\n 3. Eliminar vértice.\n 4. Eliminar arista del grafo.\n 5. Realizar un recorrido de profundidad.\n 6. Realizar un recorrido de amplitud.\n7. Obtener árbol de recubrimiento mínimo con Prim.\n 8.Obtener árbol de recubrimiento mínimo con kruskal.\n 9.Ruta mínima de un vértice a todos los demás usando Dijkstra.\n 10. Determinar la ruta mínima para llegar de cualquier vértice origen a todos los demás vértices del grafo usando Floyd-Warshal.\n11.Salir.\n 12.Ejecuta prueba\n";
    
    std::cin>>op;
    
    switch (op) {
        case 1:
            std::cout<<"Inserte el id del vértice a insertar\n";
            std::cin>>vert1;
            insertVertex(vert1, digraph);
            break;
        case 2:
            std::cout<<"Inserte el id del vértice origen de la arista\n";
            std::cin>>vert1;
            std::cout<<"Inserte el id del vértice destino de la arista\n";
            std::cin>>vert2;
            std::cout<<"Inserte el peso de la arista\n";
            std::cin>>w;
            insertEdge(vert1, vert2, w, digraph);
            break;
        case 3:
            std::cout<<"Inserte el id del vértice a eliminar\n";
            std::cin>>vert1;
            deleteVert(vert1, digraph);
            break;
        case 4:
            std::cout<<"Inserte el id del vértice origen de la arista\n";
            std::cin>>vert1;
            std::cout<<"Inserte el id del vértice destino de la arista\n";
            std::cin>>vert2;
            deleteEdge(vert1, vert2,  digraph);
            break;
        case 5:
            dfs(digraph);
            break;
        case 6:
            bfs(digraph);
            break;
        case 7:
            prim(digraph);
            break;
        case 8:
            kruskal(digraph);
            break;
        case 9:
             std::cout<<"Inserte el id del vértice del que hará el recorrido\n";
            dijkstra(digraph, vert1);
            break;
        case 10:
            floydwarshal(digraph);
            break;
        case 11:
            op=-1;
            break;
        case 12:
            ejecuta(digraph);
            break;
        default:
            break;
    }

    
    return 0;
}

template < typename DirectedGraph >
void insertVertex(std::string _id, DirectedGraph & digraph){
    
    bool found = false;
        typename graph_traits < DirectedGraph >::vertex_descriptor u;
    typename DirectedGraph::vertex_iterator begin, end;
    tie(begin, end) = vertices(digraph);
    for(;begin!=end; begin++)
    {
        if(digraph[*begin].id == _id)
        {
            found=true;
            
        }
    }
    
    if (found)
    {
        std::cout<<"Ya existe un vértice con este identificador\n";
    } else
    {
    u = add_vertex(digraph);
        digraph[u].id=_id;
    }
}
template < typename DirectedGraph >
void insertEdge(std::string vert1, std::string vert2, double w, DirectedGraph & digraph){
    
    vertex_descriptor v1, v2;
    typename DirectedGraph::vertex_iterator begin, end;
    bool found1=false, found2=false;
    
    tie(begin, end) = vertices(digraph);
    for(;begin!=end; begin++)
    {
        if(digraph[*begin].id == vert1)
        {
            v1 = *begin;
            found1=true;
            
        } else if (digraph[*begin].id == vert2)
        {
            v2= *begin;
            found2=true;
        }
    }
    
    if (found1 && found2)
    {
        add_edge(v1, v1, Weight(2), digraph);
    }else
    {
        std::cout<<"No se encontro alguno o ambos vértices\n";
    }

}
template < typename DirectedGraph >
void deleteVert(std::string vert1, DirectedGraph & digraph){
    typename graph_traits < DirectedGraph >::vertex_descriptor v1;
   typename DirectedGraph::vertex_iterator begin, end;
    bool found=false;
    
    tie(begin, end) = vertices(digraph);
    for(;begin!=end; begin++)
    {
        if(digraph[*begin].id == vert1)
        {
            v1 = *begin;
            found=true;
        }
    }
    
    if (found)
    {
        clear_vertex(v1, digraph);
        remove_vertex(v1, digraph);
    }
    
}
template < typename DirectedGraph >
void deleteEdge(std::string vert1, std::string vert2, DirectedGraph & digraph){
    vertex_descriptor v1, v2;
    typename DirectedGraph::vertex_iterator begin, end;
    bool found1=false, found2=false;
    
    tie(begin, end) = vertices(digraph);
    for(;begin!=end; begin++)
    {
        if(digraph[*begin].id == vert1)
        {
            v1 = *begin;
            found1=true;
            
        } else if (digraph[*begin].id == vert2)
        {
            v2= *begin;
            found2=true;
        }
    }
    
    if (found1 && found2)
    {
        remove_edge(v1, v2, digraph);
        
    }else
    {
        std::cout<<"No se encontro alguno o ambos vértices\n";
    }

}
template < typename DirectedGraph >
void dfs(DirectedGraph & digraph){
    custom_dfs_visitor vis;
    depth_first_search(digraph, visitor(vis));
}
template < typename DirectedGraph >
void bfs(DirectedGraph & digraph){
    custom_bfs_visitor vis;
    breadth_first_search(digraph, vertex(0,digraph), visitor(vis));
}
template < typename DirectedGraph >
void prim(DirectedGraph & digraph){
    std::vector < typename graph_traits < DirectedGraph >::vertex_descriptor > p(num_vertices(digraph));
    
    prim_minimum_spanning_tree(digraph, &p[0]);
    
    for (std::size_t i = 0; i != p.size(); ++i)
        if (p[i] != i)
            std::cout << "parent[" << i << "] = " << p[i] << std::endl;
        else
            std::cout << "parent[" << i << "] = no parent" << std::endl;
    
}
template < typename DirectedGraph >
void kruskal(DirectedGraph & digraph){
    typename property_map < DirectedGraph, edge_weight_t >::type weight = get(edge_weight, digraph);
    typedef typename graph_traits < DirectedGraph >::edge_descriptor Edge;
    
    std::vector < Edge > spanning_tree;
    
    kruskal_minimum_spanning_tree(digraph, std::back_inserter(spanning_tree));
    for (typename std::vector < Edge >::iterator ei = spanning_tree.begin();
         ei != spanning_tree.end(); ++ei) {
        std::cout << source(*ei, digraph) << " <--> " << target(*ei, digraph)
        << " with weight of " << weight[*ei]
        << std::endl;
    }
}
template < typename DirectedGraph >
void dijkstra(DirectedGraph & digraph, std::string id){
    
    bool found = false;
    typename graph_traits < DirectedGraph >::vertex_descriptor u;
    typename DirectedGraph::vertex_iterator begin, end;
    tie(begin, end) = vertices(digraph);
    for(;begin!=end; begin++)
    {
        if(digraph[*begin].id == id)
        {
            found=true;
            u=*begin;
            
        }
    }
    
    if (found)
    {
        std::vector<vertex_descriptor> p(num_vertices(digraph));
        std::vector<int> d(num_vertices(digraph));
        dijkstra_shortest_paths(digraph, u,
                                predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, digraph))).distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, digraph))));
        std::cout << "distances and parents:" << std::endl;
        
        typename DirectedGraph::vertex_iterator begin, end;
        for (boost::tie(begin, end) = vertices(digraph); begin != end; ++begin) {
            std::cout << "distance(" << digraph[*begin].id << ") = " << d[*begin] << ", ";
            std::cout << "parent(" << digraph[*begin].id << ") = " << digraph[p[*begin]].id << std::endl;
        }
        std::cout << std::endl;
        
    } else
    {
       std::cout<<"No existe el vértice\n";
    }

   
}

template <typename DirectedGraph>
void dijkstra2(DirectedGraph & digraph){
    std::vector<vertex_descriptor> p(num_vertices(digraph));
    std::vector<int> d(num_vertices(digraph));
    vertex_descriptor u = vertex(0, digraph);
    
    dijkstra_shortest_paths(digraph, u, boost::predecessor_map(&p[0]).distance_map(&d[0]));
//    
//    std::cout << "distances and parents:" << std::endl;
//    
//    typename DirectedGraph::vertex_iterator begin, end;
//    for (boost::tie(begin, end) = vertices(digraph); begin != end; ++begin) {
//        std::cout << "distance(" << digraph[*begin].id << ") = " << d[*begin] << ", ";
//        std::cout << "parent(" << digraph[*begin].id << ") = " << digraph[p[*begin]].id << std::endl;
//    }
//    std::cout << std::endl;
}
template < typename DirectedGraph >
void floydwarshal(DirectedGraph & digraph){
    WeightMap weight_pmap = get(edge_weight, digraph);
    
    DistanceMatrix distances(num_vertices(digraph));
    DistanceMatrixMap dm(distances, digraph);
    
    
    floyd_warshall_all_pairs_shortest_paths(digraph, dm ,weight_map(weight_pmap));
    
    std::cout << "Distance matrix: " << std::endl;
    for (std::size_t i = 0; i < num_vertices(digraph); ++i)
    {
        for (std::size_t j = 0; j < num_vertices(digraph); ++j)
        {
            std::cout << "From vertex " << i+1 << " to " << j+1 << " : ";
            if(distances[i][j] == std::numeric_limits<float>::max())
                std::cout << "inf" << std::endl;
            else
                std::cout << distances[i][j] << std::endl;
        }
        std::cout << std::endl;
    }
    
   }

void ejecuta(DirectedGraph & digraph){
    std::vector<vertex_descriptor> v(14);
    
    for (int i=0; i<14; i++) {
        v[i]= add_vertex(digraph);
        digraph[v[i]].id = std::to_string(i+1);
    }
    
    add_edge(v[0], v[2], Weight(8), digraph);
    add_edge(v[0], v[3], Weight(8), digraph);
    
    add_edge(v[1], v[4], Weight(7), digraph);
    
    add_edge(v[2], v[4], Weight(8), digraph);
    add_edge(v[2], v[9], Weight(4), digraph);
    add_edge(v[2], v[1], Weight(7), digraph);
    
    add_edge(v[3], v[6], Weight(3), digraph);
    add_edge(v[3], v[7], Weight(2), digraph);
    add_edge(v[3], v[4], Weight(1), digraph);
    
    add_edge(v[4], v[5], Weight(9), digraph);
    
    add_edge(v[5], v[12], Weight(4), digraph);
    
    add_edge(v[6], v[3], Weight(6), digraph);
    
    add_edge(v[7], v[6], Weight(3), digraph);
    add_edge(v[7], v[8], Weight(3), digraph);
    
    add_edge(v[8], v[9], Weight(2), digraph);
    add_edge(v[8], v[11], Weight(4), digraph);
    
    add_edge(v[9], v[2], Weight(10), digraph);
    add_edge(v[9], v[5], Weight(6), digraph);
    
    add_edge(v[10], v[11], Weight(6), digraph);
    
    add_edge(v[11], v[10], Weight(8), digraph);
    add_edge(v[11], v[8], Weight(2), digraph);
    add_edge(v[11], v[13], Weight(9), digraph);
    
    add_edge(v[12], v[13], Weight(6), digraph);
    
    add_edge(v[13], v[12], Weight(2), digraph);
    
    
    auto begin = std::chrono::high_resolution_clock::now();
    dfs(digraph);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"DFS: "<<time.count()<<std::endl;
    
    begin = std::chrono::high_resolution_clock::now();
    bfs(digraph);
    end = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"BFS: "<<time.count()<<std::endl;

    begin = std::chrono::high_resolution_clock::now();
    prim(digraph);
    end = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"Prim: "<<time.count()<<std::endl;
    
    begin = std::chrono::high_resolution_clock::now();
    kruskal(digraph);
    end = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"Kruskal: "<<time.count()<<std::endl;
    
    begin = std::chrono::high_resolution_clock::now();
    dijkstra2(digraph);
    end = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"Dijkstra: "<<time.count()<<std::endl;
    
    
    begin = std::chrono::high_resolution_clock::now();
    floydwarshal(digraph);
    end = std::chrono::high_resolution_clock::now();
    time = std::chrono::duration_cast<micro>(end-begin);
    
    std::cout<<"Floyd-Warshal: "<<time.count()<<std::endl;
    

    

    
    
}
