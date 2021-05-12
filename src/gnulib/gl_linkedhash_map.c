/* Map data type implemented by a hash table with a linked list.
   Copyright (C) 2006, 2008-2021 Free Software Foundation, Inc.
   Written by Bruno Haible <bruno@clisp.org>, 2018.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <https://www.gnu.org/licenses/>.  */

#include <config.h>

/* Specification.  */
#include "gl_linkedhash_map.h"

#include <stdint.h> /* for uintptr_t, SIZE_MAX */
#include <stdlib.h>

#include "xsize.h"

/* --------------------------- gl_map_t Data Type --------------------------- */

#include "gl_anyhash1.h"

/* Concrete list node implementation, valid for this file only.  */
struct gl_list_node_impl
{
  struct gl_hash_entry h;           /* hash table entry fields; must be first */
  struct gl_list_node_impl *next;
  struct gl_list_node_impl *prev;
  const void *key;
  const void *value;
};
typedef struct gl_list_node_impl * gl_list_node_t;

/* Concrete gl_map_impl type, valid for this file only.  */
struct gl_map_impl
{
  struct gl_map_impl_base base;
  gl_mapkey_hashcode_fn hashcode_fn;
  /* A hash table: managed as an array of collision lists.  */
  struct gl_hash_entry **table;
  size_t table_size;
  /* A circular list anchored at root.
     The first node is = root.next, the last node is = root.prev.
     The root's value is unused.  */
  struct gl_list_node_impl root;
  /* Number of list nodes, excluding the root.  */
  size_t count;
};

#define CONTAINER_T gl_map_t
#define CONTAINER_COUNT(map) (map)->count
#include "gl_anyhash2.h"

/* If the symbol SIGNAL_SAFE_MAP is defined, the code is compiled in such
   a way that a gl_map_t data structure may be used from within a signal
   handler.  The operations allowed in the signal handler are:
     gl_map_iterator, gl_map_iterator_next, gl_map_iterator_free.
   The map and node fields that are therefore accessed from the signal handler
   are:
     map->root, node->next, node->value.
   We are careful to make modifications to these fields only in an order
   that maintains the consistency of the list data structure at any moment,
   and we use 'volatile' assignments to prevent the compiler from reordering
   such assignments.  */
#ifdef SIGNAL_SAFE_MAP
# define ASYNCSAFE(type) *(type volatile *)&
#else
# define ASYNCSAFE(type)
#endif

/* --------------------------- gl_map_t Data Type --------------------------- */

static gl_map_t
gl_linkedhash_nx_create_empty (gl_map_implementation_t implementation,
                               gl_mapkey_equals_fn equals_fn,
                               gl_mapkey_hashcode_fn hashcode_fn,
                               gl_mapkey_dispose_fn kdispose_fn,
                               gl_mapvalue_dispose_fn vdispose_fn)
{
  struct gl_map_impl *map =
    (struct gl_map_impl *) malloc (sizeof (struct gl_map_impl));

  if (map == NULL)
    return NULL;

  map->base.vtable = implementation;
  map->base.equals_fn = equals_fn;
  map->base.kdispose_fn = kdispose_fn;
  map->base.vdispose_fn = vdispose_fn;
  map->hashcode_fn = hashcode_fn;
  map->table_size = 11;
  map->table =
    (gl_hash_entry_t *) calloc (map->table_size, sizeof (gl_hash_entry_t));
  if (map->table == NULL)
    goto fail;
  map->root.next = &map->root;
  map->root.prev = &map->root;
  map->count = 0;

  return map;

 fail:
  free (map);
  return NULL;
}

static size_t _GL_ATTRIBUTE_PURE
gl_linkedhash_size (gl_map_t map)
{
  return map->count;
}

static bool _GL_ATTRIBUTE_PURE
gl_linkedhash_search (gl_map_t map, const void *key, const void **valuep)
{
  size_t hashcode =
    (map->hashcode_fn != NULL
     ? map->hashcode_fn (key)
     : (size_t)(uintptr_t) key);
  size_t bucket = hashcode % map->table_size;
  gl_mapkey_equals_fn equals = map->base.equals_fn;

  /* Look for a match in the hash bucket.  */
  gl_list_node_t node;

  for (node = (gl_list_node_t) map->table[bucket];
       node != NULL;
       node = (gl_list_node_t) node->h.hash_next)
    if (node->h.hashcode == hashcode
        && (equals != NULL
            ? equals (key, node->key)
            : key == node->key))
      {
        *valuep = node->value;
        return true;
      }
  return false;
}

static int
gl_linkedhash_nx_getput (gl_map_t map, const void *key, const void *value,
                         const void **oldvaluep)
{
  size_t hashcode =
    (map->hashcode_fn != NULL
     ? map->hashcode_fn (key)
     : (size_t)(uintptr_t) key);
  size_t bucket = hashcode % map->table_size;
  gl_mapkey_equals_fn equals = map->base.equals_fn;

  /* Look for a match in the hash bucket.  */
  {
    gl_list_node_t node;

    for (node = (gl_list_node_t) map->table[bucket];
         node != NULL;
         node = (gl_list_node_t) node->h.hash_next)
      if (node->h.hashcode == hashcode
          && (equals != NULL
              ? equals (key, node->key)
              : key == node->key))
        {
          *oldvaluep = node->value;
          node->value = value;
          return 0;
        }
  }

  /* Allocate a new node.  */
  gl_list_node_t node =
    (struct gl_list_node_impl *) malloc (sizeof (struct gl_list_node_impl));

  if (node == NULL)
    return -1;

  ASYNCSAFE(const void *) node->key = key;
  ASYNCSAFE(const void *) node->value = value;
  node->h.hashcode = hashcode;

  /* Add node to the hash table.  */
  node->h.hash_next = map->table[bucket];
  map->table[bucket] = &node->h;

  /* Add node to the map.  */
  ASYNCSAFE(gl_list_node_t) node->next = &map->root;
  node->prev = map->root.prev;
  ASYNCSAFE(gl_list_node_t) node->prev->next = node;
  map->root.prev = node;
  map->count++;

  hash_resize_after_add (map);

  return 1;
}

static bool
gl_linkedhash_getremove (gl_map_t map, const void *key, const void **oldvaluep)
{
  size_t hashcode =
    (map->hashcode_fn != NULL
     ? map->hashcode_fn (key)
     : (size_t)(uintptr_t) key);
  size_t bucket = hashcode % map->table_size;
  gl_mapkey_equals_fn equals = map->base.equals_fn;

  /* Look for the first match in the hash bucket.  */
  gl_list_node_t *nodep;

  for (nodep = (gl_list_node_t *) &map->table[bucket];
       *nodep != NULL;
       nodep = (gl_list_node_t *) &(*nodep)->h.hash_next)
    {
      gl_list_node_t node = *nodep;
      if (node->h.hashcode == hashcode
          && (equals != NULL
              ? equals (key, node->key)
              : key == node->key))
        {
          *oldvaluep = node->value;

          /* Remove node from the hash table.  */
          *nodep = (gl_list_node_t) node->h.hash_next;

          /* Remove node from the list.  */
          {
            gl_list_node_t prev = node->prev;
            gl_list_node_t next = node->next;

            ASYNCSAFE(gl_list_node_t) prev->next = next;
            next->prev = prev;
          }
          map->count--;

          if (map->base.kdispose_fn != NULL)
            map->base.kdispose_fn (node->key);
          free (node);
          return true;
        }
    }

  return false;
}

static void
gl_linkedhash_free (gl_map_t map)
{
  gl_mapkey_dispose_fn kdispose = map->base.kdispose_fn;
  gl_mapvalue_dispose_fn vdispose = map->base.vdispose_fn;
  gl_list_node_t node;

  for (node = map->root.next; node != &map->root; )
    {
      gl_list_node_t next = node->next;
      if (vdispose != NULL)
        vdispose (node->value);
      if (kdispose != NULL)
        kdispose (node->key);
      free (node);
      node = next;
    }
  free (map->table);
  free (map);
}

/* ---------------------- gl_map_iterator_t Data Type ---------------------- */

/* Iterate through the list, not through the hash buckets, so that the order
   in which the pairs are returned is predictable.  */

static gl_map_iterator_t
gl_linkedhash_iterator (gl_map_t map)
{
  gl_map_iterator_t result;

  result.vtable = map->base.vtable;
  result.map = map;
  result.p = map->root.next;
  result.q = &map->root;
#if defined GCC_LINT || defined lint
  result.i = 0;
  result.j = 0;
  result.count = 0;
#endif

  return result;
}

static bool
gl_linkedhash_iterator_next (gl_map_iterator_t *iterator,
                             const void **keyp, const void **valuep)
{
  if (iterator->p != iterator->q)
    {
      gl_list_node_t node = (gl_list_node_t) iterator->p;
      *keyp = node->key;
      *valuep = node->value;
      iterator->p = node->next;
      return true;
    }
  else
    return false;
}

static void
gl_linkedhash_iterator_free (gl_map_iterator_t *iterator)
{
}


const struct gl_map_implementation gl_linkedhash_map_implementation =
  {
    gl_linkedhash_nx_create_empty,
    gl_linkedhash_size,
    gl_linkedhash_search,
    gl_linkedhash_nx_getput,
    gl_linkedhash_getremove,
    gl_linkedhash_free,
    gl_linkedhash_iterator,
    gl_linkedhash_iterator_next,
    gl_linkedhash_iterator_free
  };
