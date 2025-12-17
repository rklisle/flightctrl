#ifndef __SLIST_H__
#define __SLIST_H__
#include <stddef.h>

/**
 * Single List structure
 */
struct slist_node
{
    struct slist_node *next;            /**< point to next node. */
};

typedef struct slist_node slist_t;      /**< Type for single list. */
/**
 * @brief initialize a single list
 *
 * @param l the single list to be initialized
 */
inline void slist_init(slist_t *l)
{
    l->next = NULL;
}

inline void slist_append(slist_t *l, slist_t *n)
{
    struct slist_node *node;

    node = l;
    while (node->next) node = node->next;

    /* append the node to the tail */
    node->next = n;
    n->next = NULL;
}

inline void slist_insert(slist_t *l, slist_t *n)
{
    n->next = l->next;
    l->next = n;
}

inline size_t slist_len(const slist_t *l)
{
    size_t len = 0;
    const slist_t *list = l->next;
    while (list != NULL)
    {
        list = list->next;
        len ++;
    }

    return len;
}

inline slist_t *slist_pop(slist_t *l)
{
    struct slist_node *node = l;

    /* remove node */
    node = node->next;
    if (node != (slist_t *)0)
    {
        ((struct slist_node *)l)->next = node->next;
    }

    return node;
}

inline slist_t *slist_remove(slist_t *l, slist_t *n)
{
    /* remove slist head */
    struct slist_node *node = l;
    while (node->next && node->next != n) node = node->next;

    /* remove node */
    if (node->next != (slist_t *)0) node->next = node->next->next;

    return l;
}

inline slist_t *slist_first(slist_t *l)
{
    return l->next;
}

inline slist_t *slist_tail(slist_t *l)
{
    while (l->next) l = l->next;

    return l;
}

inline slist_t *slist_next(slist_t *n)
{
    return n->next;
}

inline int slist_isempty(slist_t *l)
{
    return l->next == NULL;
}

/**
 * @brief get the struct for this single list node
 * @param node the entry point
 * @param type the type of structure
 * @param member the name of list in structure
 */
#define slist_entry(node, type, member) \
    ((type *)((char *)(node) - (unsigned long)(&((type *)0)->member)))

/**
 * slist_for_each - iterate over a single list
 * @param pos the slist_t * to use as a loop cursor.
 * @param head the head for your single list.
 */
#define slist_for_each(pos, head) \
    for (pos = (head)->next; pos != NULL; pos = pos->next)

/**
 * slist_for_each_entry  -   iterate over single list of given type
 * @param pos the type * to use as a loop cursor.
 * @param head the head for your single list.
 * @param member the name of the list_struct within the struct.
 */
#define slist_for_each_entry(pos, head, member) \
    for (pos = ((head)->next == (NULL) ? (NULL) : slist_entry((head)->next, rt_typeof(*pos), member)); \
         pos != (NULL) && &pos->member != (NULL); \
         pos = (pos->member.next == (NULL) ? (NULL) : slist_entry(pos->member.next, rt_typeof(*pos), member)))

/**
 * slist_first_entry - get the first element from a slist
 * @param ptr the slist head to take the element from.
 * @param type the type of the struct this is embedded in.
 * @param member the name of the slist_struct within the struct.
 *
 * Note, that slist is expected to be not empty.
 */
#define slist_first_entry(ptr, type, member) \
    slist_entry((ptr)->next, type, member)

/**
 * slist_tail_entry - get the tail element from a slist
 * @param ptr the slist head to take the element from.
 * @param type the type of the struct this is embedded in.
 * @param member the name of the slist_struct within the struct.
 *
 * Note, that slist is expected to be not empty.
 */
#define slist_tail_entry(ptr, type, member) \
    slist_entry(slist_tail(ptr), type, member)


#endif