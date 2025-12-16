SELECT
    s_acctbal,
    s_name,
    n_name,
    p_partkey,
    p_mfgr,
    s_address,
    s_phone,
    s_comment
FROM
    '/nvme/tpch/part.parquet',
    '/nvme/tpch/supplier.parquet',
    '/nvme/tpch/partsupp.parquet',
    '/nvme/tpch/nation.parquet',
    '/nvme/tpch/region.parquet'
WHERE
    p_partkey = ps_partkey
    AND s_suppkey = ps_suppkey
    AND p_size = 15
    AND p_type LIKE '%BRASS'
    AND s_nationkey = n_nationkey
    AND n_regionkey = r_regionkey
    AND r_name = 'EUROPE'
    AND ps_supplycost = (
        SELECT
            min(ps_supplycost)
        FROM
            '/nvme/tpch/partsupp.parquet',
            '/nvme/tpch/supplier.parquet',
            '/nvme/tpch/nation.parquet',
            '/nvme/tpch/region.parquet'
        WHERE
            p_partkey = ps_partkey
            AND s_suppkey = ps_suppkey
            AND s_nationkey = n_nationkey
            AND n_regionkey = r_regionkey
            AND r_name = 'EUROPE')
ORDER BY
    s_acctbal DESC,
    n_name,
    s_name,
    p_partkey
LIMIT 100;
SELECT
    s_acctbal,
    s_name,
    n_name,
    p_partkey,
    p_mfgr,
    s_address,
    s_phone,
    s_comment
FROM
    '/nvme/tpch/part.parquet',
    '/nvme/tpch/supplier.parquet',
    '/nvme/tpch/partsupp.parquet',
    '/nvme/tpch/nation.parquet',
    '/nvme/tpch/region.parquet'
WHERE
    p_partkey = ps_partkey
    AND s_suppkey = ps_suppkey
    AND p_size = 15
    AND p_type LIKE '%BRASS'
    AND s_nationkey = n_nationkey
    AND n_regionkey = r_regionkey
    AND r_name = 'EUROPE'
    AND ps_supplycost = (
        SELECT
            min(ps_supplycost)
        FROM
            '/nvme/tpch/partsupp.parquet',
            '/nvme/tpch/supplier.parquet',
            '/nvme/tpch/nation.parquet',
            '/nvme/tpch/region.parquet'
        WHERE
            p_partkey = ps_partkey
            AND s_suppkey = ps_suppkey
            AND s_nationkey = n_nationkey
            AND n_regionkey = r_regionkey
            AND r_name = 'EUROPE')
ORDER BY
    s_acctbal DESC,
    n_name,
    s_name,
    p_partkey
LIMIT 100;
SELECT
    s_acctbal,
    s_name,
    n_name,
    p_partkey,
    p_mfgr,
    s_address,
    s_phone,
    s_comment
FROM
    '/nvme/tpch/part.parquet',
    '/nvme/tpch/supplier.parquet',
    '/nvme/tpch/partsupp.parquet',
    '/nvme/tpch/nation.parquet',
    '/nvme/tpch/region.parquet'
WHERE
    p_partkey = ps_partkey
    AND s_suppkey = ps_suppkey
    AND p_size = 15
    AND p_type LIKE '%BRASS'
    AND s_nationkey = n_nationkey
    AND n_regionkey = r_regionkey
    AND r_name = 'EUROPE'
    AND ps_supplycost = (
        SELECT
            min(ps_supplycost)
        FROM
            '/nvme/tpch/partsupp.parquet',
            '/nvme/tpch/supplier.parquet',
            '/nvme/tpch/nation.parquet',
            '/nvme/tpch/region.parquet'
        WHERE
            p_partkey = ps_partkey
            AND s_suppkey = ps_suppkey
            AND s_nationkey = n_nationkey
            AND n_regionkey = r_regionkey
            AND r_name = 'EUROPE')
ORDER BY
    s_acctbal DESC,
    n_name,
    s_name,
    p_partkey
LIMIT 100;
