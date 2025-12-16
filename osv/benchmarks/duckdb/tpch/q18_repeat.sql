SELECT
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice,
    sum(l_quantity)
FROM
    '/nvme/tpch/customer.parquet',
    '/nvme/tpch/orders.parquet',
    '/nvme/tpch/lineitem.parquet'
WHERE
    o_orderkey IN (
        SELECT
            l_orderkey
        FROM
            '/nvme/tpch/lineitem.parquet'
        GROUP BY
            l_orderkey
        HAVING
            sum(l_quantity) > 300)
    AND c_custkey = o_custkey
    AND o_orderkey = l_orderkey
GROUP BY
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice
ORDER BY
    o_totalprice DESC,
    o_orderdate
LIMIT 100;
SELECT
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice,
    sum(l_quantity)
FROM
    '/nvme/tpch/customer.parquet',
    '/nvme/tpch/orders.parquet',
    '/nvme/tpch/lineitem.parquet'
WHERE
    o_orderkey IN (
        SELECT
            l_orderkey
        FROM
            '/nvme/tpch/lineitem.parquet'
        GROUP BY
            l_orderkey
        HAVING
            sum(l_quantity) > 300)
    AND c_custkey = o_custkey
    AND o_orderkey = l_orderkey
GROUP BY
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice
ORDER BY
    o_totalprice DESC,
    o_orderdate
LIMIT 100;
SELECT
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice,
    sum(l_quantity)
FROM
    '/nvme/tpch/customer.parquet',
    '/nvme/tpch/orders.parquet',
    '/nvme/tpch/lineitem.parquet'
WHERE
    o_orderkey IN (
        SELECT
            l_orderkey
        FROM
            '/nvme/tpch/lineitem.parquet'
        GROUP BY
            l_orderkey
        HAVING
            sum(l_quantity) > 300)
    AND c_custkey = o_custkey
    AND o_orderkey = l_orderkey
GROUP BY
    c_name,
    c_custkey,
    o_orderkey,
    o_orderdate,
    o_totalprice
ORDER BY
    o_totalprice DESC,
    o_orderdate
LIMIT 100;
